#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_mac.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

// I2C Configuration
#define I2C_MASTER_SCL_IO           9
#define I2C_MASTER_SDA_IO           8
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TIMEOUT_MS       1000

// Vibration Motor Configuration
#define MOTOR_GPIO                  3
#define ANGLE_THRESHOLD             45.0f

// LSM6DSOX Configuration
#define LSM6DSOX_ADDR               0x6A
#define WHO_AM_I_REG                0x0F
#define CTRL1_XL                    0x10
#define CTRL3_C                     0x12
#define OUTX_L_A                    0x28

// Step Detection Configuration
#define ACCEL_SCALE                 0.061f
#define STEP_THRESHOLD              1100.0f
#define STEP_MIN_INTERVAL_MS        250
#define AVG_STRIDE_LENGTH_M         0.7f

// HX711 Configuration
#define HX711_DOUT_GPIO             6
#define HX711_SCK_GPIO              5
#define HX711_SCALE_FACTOR          107.85f
#define HX711_OFFSET                -109700
#define HX711_SAMPLES               1
#define KG_TO_LBS                   2.20462f

// Bluetooth Configuration
#define BLE_DEVICE_NAME_MAX_LEN     32
#define DEFAULT_DEVICE_NAME         "SMARTHANDLE_L"

// UUIDs
static const ble_uuid16_t gatt_svc_uuid         = BLE_UUID16_INIT(0xFFE0);
static const ble_uuid16_t gatt_char_weight_uuid = BLE_UUID16_INIT(0xFFE2);
static const ble_uuid16_t gatt_char_accel_uuid  = BLE_UUID16_INIT(0xFFE8);

// Pedometer state
typedef struct {
    uint32_t step_count;
    float last_accel_mag;
    int64_t last_step_time_us;
    float speed_mps;
    bool step_detected;
} pedometer_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} accel_raw_t;

static pedometer_t pedometer = {0};
static accel_raw_t latest_accel = {0};
static float latest_weight_lbs = 0.0f;
static const char *TAG = "LSM6DSOX";
static const char *BLE_TAG = "BLE";
static const char *HX_TAG = "HX711";

// BLE state
static bool ble_connected = false;
static uint16_t ble_conn_handle = 0;
static char device_name[BLE_DEVICE_NAME_MAX_LEN] = DEFAULT_DEVICE_NAME;
static uint16_t accel_char_handle = 0;
static uint16_t weight_char_handle = 0;

// I2C handles
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static i2c_master_dev_handle_t lsm6dsox_handle = NULL;

// Forward declarations
static void ble_app_advertise(void);
static int gatt_char_access(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg);
static int gatt_dsc_access(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg);

// ─── GATT Service Definition ─────────────────────────────────────────────────

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &gatt_char_weight_uuid.u,
                .access_cb = gatt_char_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &weight_char_handle,
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    {
                        .uuid = BLE_UUID16_DECLARE(0x2902),
                        .att_flags = BLE_ATT_F_READ | BLE_ATT_F_WRITE,
                        .access_cb = gatt_dsc_access,
                    },
                    { 0 }
                },
            },
            {
                .uuid = &gatt_char_accel_uuid.u,
                .access_cb = gatt_char_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &accel_char_handle,
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    {
                        .uuid = BLE_UUID16_DECLARE(0x2902),
                        .att_flags = BLE_ATT_F_READ | BLE_ATT_F_WRITE,
                        .access_cb = gatt_dsc_access,
                    },
                    { 0 }
                },
            },
            { 0 }
        },
    },
    { 0 }
};

static int gatt_dsc_access(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    return 0;
}

static int gatt_char_access(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return 0;

    if (attr_handle == weight_char_handle) {
        uint16_t weight_int = (uint16_t)(latest_weight_lbs * 100);
        os_mbuf_append(ctxt->om, &weight_int, sizeof(weight_int));
    } else if (attr_handle == accel_char_handle) {
        uint8_t buf[6];
        memcpy(buf,     &latest_accel.x, 2);
        memcpy(buf + 2, &latest_accel.y, 2);
        memcpy(buf + 4, &latest_accel.z, 2);
        os_mbuf_append(ctxt->om, buf, sizeof(buf));
    }

    return 0;
}

// ─── BLE Notify Helpers ───────────────────────────────────────────────────────

static void ble_notify_accel(void)
{
    if (!ble_connected || accel_char_handle == 0) return;
    uint8_t buf[6];
    memcpy(buf,     &latest_accel.x, 2);
    memcpy(buf + 2, &latest_accel.y, 2);
    memcpy(buf + 4, &latest_accel.z, 2);
    struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, sizeof(buf));
    if (om) ble_gatts_notify_custom(ble_conn_handle, accel_char_handle, om);
}

static void ble_notify_weight(void)
{
    if (!ble_connected || weight_char_handle == 0) return;
    uint16_t weight_int = (uint16_t)(latest_weight_lbs * 100);
    struct os_mbuf *om = ble_hs_mbuf_from_flat(&weight_int, sizeof(weight_int));
    if (om) ble_gatts_notify_custom(ble_conn_handle, weight_char_handle, om);
}

// ─── HX711 ───────────────────────────────────────────────────────────────────

static void hx711_init(void)
{
    gpio_config_t io_conf = {0};

    io_conf.pin_bit_mask = (1ULL << HX711_DOUT_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << HX711_SCK_GPIO);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    gpio_set_level(HX711_SCK_GPIO, 0);
    ESP_LOGI(HX_TAG, "HX711 initialized on DOUT=GPIO%d SCK=GPIO%d",
             HX711_DOUT_GPIO, HX711_SCK_GPIO);
}

static bool hx711_is_ready(void)
{
    return gpio_get_level(HX711_DOUT_GPIO) == 0;
}

static int32_t hx711_read_raw(void)
{
    uint32_t timeout = 200;
    while (!hx711_is_ready()) {
        vTaskDelay(1 / portTICK_PERIOD_MS);
        if (--timeout == 0) {
            ESP_LOGW(HX_TAG, "HX711 timeout");
            return INT32_MIN;
        }
    }

    uint32_t raw = 0;

    for (int i = 0; i < 24; i++) {
        gpio_set_level(HX711_SCK_GPIO, 1);
        esp_rom_delay_us(1);
        raw = (raw << 1) | gpio_get_level(HX711_DOUT_GPIO);
        gpio_set_level(HX711_SCK_GPIO, 0);
        esp_rom_delay_us(1);
    }

    gpio_set_level(HX711_SCK_GPIO, 1);
    esp_rom_delay_us(1);
    gpio_set_level(HX711_SCK_GPIO, 0);
    esp_rom_delay_us(1);

    if (raw & 0x800000) {
        raw |= 0xFF000000;
    }

    return (int32_t)raw;
}

static float hx711_read_lbs(void)
{
    int64_t sum = 0;
    int valid_count = 0;
    for (int i = 0; i < HX711_SAMPLES; i++) {
        int32_t raw = hx711_read_raw();
        if (raw != INT32_MIN) {
            sum += raw;
            valid_count++;
        }
    }
    if (valid_count == 0) return latest_weight_lbs;

    int32_t avg = (int32_t)(sum / valid_count);
    printf("  [HX711 RAW: %ld]\n", (long)avg);

    float grams = (float)(avg - HX711_OFFSET) / HX711_SCALE_FACTOR;
    if (grams < 0.0f) grams = 0.0f;
    return (grams / 1000.0f) * KG_TO_LBS;
}

// ─── I2C ─────────────────────────────────────────────────────────────────────

static esp_err_t i2c_master_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_config, &i2c_bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus creation failed: %s", esp_err_to_name(err));
        return err;
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LSM6DSOX_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    err = i2c_master_bus_add_device(i2c_bus_handle, &dev_config, &lsm6dsox_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add LSM6DSOX device: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

// ─── LSM6DSOX ────────────────────────────────────────────────────────────────

static esp_err_t lsm6dsox_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t write_buf[2] = {reg, data};
    return i2c_master_transmit(lsm6dsox_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS);
}

static esp_err_t lsm6dsox_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(lsm6dsox_handle, &reg, 1, data, len, I2C_MASTER_TIMEOUT_MS);
}

static esp_err_t lsm6dsox_init(void)
{
    uint8_t who_am_i;
    esp_err_t ret;

    ret = lsm6dsox_read_reg(WHO_AM_I_REG, &who_am_i, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register");
        return ret;
    }

    if (who_am_i != 0x6C) {
        ESP_LOGE(TAG, "Wrong device ID: 0x%02X (expected 0x6C)", who_am_i);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "LSM6DSOX detected!");

    ret = lsm6dsox_write_reg(CTRL3_C, 0x01);
    if (ret != ESP_OK) return ret;
    vTaskDelay(50 / portTICK_PERIOD_MS);

    ret = lsm6dsox_write_reg(CTRL1_XL, 0x40);
    if (ret != ESP_OK) return ret;
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "LSM6DSOX initialized successfully");
    return ESP_OK;
}

// ─── Pedometer ───────────────────────────────────────────────────────────────

static void detect_step(float accel_mag)
{
    int64_t current_time_us = esp_timer_get_time();
    pedometer.step_detected = false;

    if (accel_mag > STEP_THRESHOLD && pedometer.last_accel_mag <= STEP_THRESHOLD) {
        int64_t time_diff_us = current_time_us - pedometer.last_step_time_us;

        if (time_diff_us > (STEP_MIN_INTERVAL_MS * 1000)) {
            pedometer.step_count++;
            pedometer.step_detected = true;

            if (pedometer.last_step_time_us > 0) {
                float time_between_steps_s = time_diff_us / 1000000.0f;
                pedometer.speed_mps = AVG_STRIDE_LENGTH_M / time_between_steps_s;
                if (pedometer.speed_mps > 2.5f) pedometer.speed_mps = 2.5f;
            }

            pedometer.last_step_time_us = current_time_us;
        }
    }

    if ((current_time_us - pedometer.last_step_time_us) > 2000000) {
        pedometer.speed_mps = 0.0f;
    }

    pedometer.last_accel_mag = accel_mag;
}

static void process_accelerometer(void)
{
    uint8_t data[6];
    int16_t accel_x, accel_y, accel_z;
    float angle_x, angle_y;

    esp_err_t ret = lsm6dsox_read_reg(OUTX_L_A, data, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read accelerometer data");
        return;
    }

    accel_x = (int16_t)(data[0] | (data[1] << 8));
    accel_y = (int16_t)(data[2] | (data[3] << 8));
    accel_z = (int16_t)(data[4] | (data[5] << 8));

    latest_accel.x = accel_x;
    latest_accel.y = accel_y;
    latest_accel.z = accel_z;

    float accel_mag = sqrtf((float)accel_x * accel_x +
                            (float)accel_y * accel_y +
                            (float)accel_z * accel_z) * ACCEL_SCALE;

    detect_step(accel_mag);

    angle_x = atan2f((float)accel_y, sqrtf((float)accel_x * accel_x + (float)accel_z * accel_z)) * 180.0f / M_PI;
    angle_y = atan2f(-(float)accel_x, sqrtf((float)accel_y * accel_y + (float)accel_z * accel_z)) * 180.0f / M_PI;

    if (fabsf(angle_x) > ANGLE_THRESHOLD || fabsf(angle_y) > ANGLE_THRESHOLD) {
        gpio_set_level(MOTOR_GPIO, 1);
    } else {
        gpio_set_level(MOTOR_GPIO, 0);
    }

    printf("Steps: %lu | Speed: %.2f m/s (%.2f km/h) | Angles: X=%.1f Y=%.1f | Right Handle: %.2f lbs",
           (unsigned long)pedometer.step_count,
           pedometer.speed_mps,
           pedometer.speed_mps * 3.6f,
           angle_x, angle_y,
           latest_weight_lbs);

    if (pedometer.step_detected) printf(" [STEP!]");
    if (ble_connected)           printf(" [BLE CONNECTED]");
    printf("\n");

    ble_notify_accel();
    ble_notify_weight();
}

// ─── BLE (NimBLE) ────────────────────────────────────────────────────────────

static int gap_event_handler(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ESP_LOGI(BLE_TAG, "Device connected");
                ble_connected = true;
                ble_conn_handle = event->connect.conn_handle;
            } else {
                ble_app_advertise();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(BLE_TAG, "Device disconnected, reason: %d", event->disconnect.reason);
            ble_connected = false;
            ble_app_advertise();
            break;

        default:
            break;
    }
    return 0;
}

static void ble_app_advertise(void)
{
    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(BLE_TAG, "Failed to set adv fields: %d", rc);
        return;
    }

    struct ble_gap_adv_params adv_params = {0};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(BLE_TAG, "Failed to start advertising: %d", rc);
    }
}

static void ble_app_on_sync(void)
{
    ble_hs_util_ensure_addr(0);
    ble_app_advertise();
    ESP_LOGI(BLE_TAG, "BLE advertising started: %s", device_name);
}

static void nimble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static esp_err_t bluetooth_init(void)
{
    esp_log_level_set("NimBLE", ESP_LOG_WARN);
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    nimble_port_init();
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);
    ble_svc_gap_device_name_set(device_name);
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    nimble_port_freertos_init(nimble_host_task);

    ESP_LOGI(BLE_TAG, "NimBLE initialized, device name: %s", device_name);
    return ESP_OK;
}

// ─── Weight Task ─────────────────────────────────────────────────────────────

static void weight_task(void *arg)
{
    while (1) {
        latest_weight_lbs = hx711_read_lbs();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// ─── Main ────────────────────────────────────────────────────────────────────

void app_main(void)
{
    ESP_LOGI(TAG, "Starting SMARTHANDLE_R on ESP32-C3");

    gpio_reset_pin(MOTOR_GPIO);
    gpio_set_direction(MOTOR_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(MOTOR_GPIO, 0);

    hx711_init();

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized on GPIO%d (SDA) and GPIO%d (SCL)",
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);

    ESP_ERROR_CHECK(lsm6dsox_init());
    ESP_ERROR_CHECK(bluetooth_init());

    xTaskCreate(weight_task, "weight_task", 2048, NULL, 5, NULL);

    printf("\n=== SMARTHANDLE_R ===\n");
    printf("I2C:    SDA=GPIO%d, SCL=GPIO%d\n", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    printf("Motor:  GPIO%d\n", MOTOR_GPIO);
    printf("HX711:  DOUT=GPIO%d, SCK=GPIO%d\n", HX711_DOUT_GPIO, HX711_SCK_GPIO);
    printf("BLE:    %s\n", device_name);
    printf("Weight: Right handle only, output in lbs\n");
    printf("====================\n");
    printf("NOTE: Adjust HX711_SCALE_FACTOR and HX711_OFFSET after calibration.\n");
    printf("====================\n\n");

    while (1) {
        process_accelerometer();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
