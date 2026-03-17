#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"  // Updated for ESP-IDF v5.0+
#include "driver/gpio.h"        // Updated for ESP-IDF v5.0+
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_mac.h"

// I2C Configuration for ESP32-C3-DevKitM-1
// Using GPIO8 (SDA) and GPIO9 (SCL) - recommended I2C pins for C3
#define I2C_MASTER_SCL_IO           21
#define I2C_MASTER_SDA_IO           22
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

// Vibration Motor Configuration
// GPIO3 is a safe choice for ESP32-C3 (avoid GPIO2 - strapping pin)
#define MOTOR_GPIO                  2
#define ANGLE_THRESHOLD             45.0

// LSM6DSOX Configuration
#define LSM6DSOX_ADDR               0x6A
#define WHO_AM_I_REG                0x0F
#define CTRL1_XL                    0x10
#define CTRL3_C                     0x12
#define OUTX_L_A                    0x28

// Step Detection Configuration
#define ACCEL_SCALE                 0.061  // mg/LSB for ±2g range
#define STEP_THRESHOLD              1100.0 // mg (acceleration threshold for step)
#define STEP_MIN_INTERVAL_MS        250    // Minimum time between steps (ms)
#define AVG_STRIDE_LENGTH_M         0.7    // Average stride length in meters

// Bluetooth Configuration
#define BLE_DEVICE_NAME_MAX_LEN     32
#define DEFAULT_DEVICE_NAME         "C3-Pedometer"
#define GATTS_SERVICE_UUID          0x180D  // Heart Rate Service (repurposed for pedometer)
#define GATTS_CHAR_UUID_STEPS       0x2A37  // Characteristic for step count
#define GATTS_CHAR_UUID_SPEED       0x2A38  // Characteristic for speed
#define GATTS_CHAR_UUID_NAME        0x2A39  // Characteristic for device name (writable)
#define GATTS_NUM_HANDLE            8

// Pedometer state
typedef struct {
    uint32_t step_count;
    float last_accel_mag;
    int64_t last_step_time_us;
    float speed_mps;  // Speed in meters per second
    bool step_detected;
} pedometer_t;

// BLE state
typedef struct {
    char device_name[BLE_DEVICE_NAME_MAX_LEN];
    uint8_t ble_addr[6];
    bool connected;
    uint16_t gatts_if;
    uint16_t conn_id;
    uint16_t handle_table[GATTS_NUM_HANDLE];
} ble_state_t;

static pedometer_t pedometer = {0};
static ble_state_t ble_state = {0};
static const char *TAG = "LSM6DSOX";
static const char *BLE_TAG = "BLE";

// Function prototypes
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

// I2C handle for ESP-IDF v5.0+
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static i2c_master_dev_handle_t lsm6dsox_handle = NULL;

// Initialize I2C master
static esp_err_t i2c_master_init(void)
{
    // Configure I2C bus
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
    
    // Add LSM6DSOX device to the bus
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

// Write a byte to a register
static esp_err_t lsm6dsox_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t write_buf[2] = {reg, data};
    return i2c_master_transmit(lsm6dsox_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS);
}

// Read bytes from a register
static esp_err_t lsm6dsox_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(lsm6dsox_handle, &reg, 1, data, len, I2C_MASTER_TIMEOUT_MS);
}

// Initialize LSM6DSOX sensor
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

// Detect step based on acceleration magnitude
static void detect_step(float accel_mag)
{
    int64_t current_time_us = esp_timer_get_time();
    pedometer.step_detected = false;
    
    // Detect rising edge (acceleration crosses threshold)
    if (accel_mag > STEP_THRESHOLD && pedometer.last_accel_mag <= STEP_THRESHOLD) {
        // Check minimum time between steps
        int64_t time_diff_us = current_time_us - pedometer.last_step_time_us;
        
        if (time_diff_us > (STEP_MIN_INTERVAL_MS * 1000)) {
            pedometer.step_count++;
            pedometer.step_detected = true;
            
            // Calculate speed based on step frequency
            // Speed = stride_length / time_between_steps
            if (pedometer.last_step_time_us > 0) {
                float time_between_steps_s = time_diff_us / 1000000.0;
                pedometer.speed_mps = AVG_STRIDE_LENGTH_M / time_between_steps_s;
                
                // Sanity check: walking speed typically 0.5 - 2.5 m/s
                if (pedometer.speed_mps > 2.5) {
                    pedometer.speed_mps = 2.5;
                }
            }
            
            pedometer.last_step_time_us = current_time_us;
        }
    }
    
    // Decay speed if no steps detected recently (stopped walking)
    int64_t time_since_last_step_us = current_time_us - pedometer.last_step_time_us;
    if (time_since_last_step_us > 2000000) {  // 2 seconds
        pedometer.speed_mps = 0.0;
    }
    
    pedometer.last_accel_mag = accel_mag;
}

// Read accelerometer data and process
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
    
    // Calculate acceleration magnitude in mg
    float accel_mag = sqrt((float)accel_x * accel_x + 
                          (float)accel_y * accel_y + 
                          (float)accel_z * accel_z) * ACCEL_SCALE;
    
    // Detect steps
    detect_step(accel_mag);
    
    // Calculate tilt angles
    angle_x = atan2(accel_y, sqrt((long)accel_x * accel_x + (long)accel_z * accel_z)) * 180.0 / M_PI;
    angle_y = atan2(-accel_x, sqrt((long)accel_y * accel_y + (long)accel_z * accel_z)) * 180.0 / M_PI;
    
    // Control motor based on angle
    if (fabs(angle_x) > ANGLE_THRESHOLD || fabs(angle_y) > ANGLE_THRESHOLD) {
        gpio_set_level(MOTOR_GPIO, 1);
    } else {
        gpio_set_level(MOTOR_GPIO, 0);
    }
    
    // Print status
    printf("Steps: %lu | Speed: %.2f m/s (%.2f km/h) | Angles: X=%.1f° Y=%.1f°", 
           (unsigned long)pedometer.step_count,
           pedometer.speed_mps,
           pedometer.speed_mps * 3.6,  // Convert to km/h
           angle_x, angle_y);
    
    if (pedometer.step_detected) {
        printf(" [STEP!]");
    }
    
    if (ble_state.connected) {
        printf(" [BLE CONNECTED]");
    }
    
    printf("\n");
}

// GAP event handler
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(&(esp_ble_adv_params_t){
                .adv_int_min = 0x20,
                .adv_int_max = 0x40,
                .adv_type = ADV_TYPE_IND,
                .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
                .channel_map = ADV_CHNL_ALL,
                .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
            });
            break;
        
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(BLE_TAG, "Advertising started successfully");
            } else {
                ESP_LOGE(BLE_TAG, "Advertising start failed");
            }
            break;
        
        default:
            break;
    }
}

// GATT server event handler
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(BLE_TAG, "GATT server registered, app_id: %d", param->reg.app_id);
            ble_state.gatts_if = gatts_if;
            
            // Set device name
            esp_ble_gap_set_device_name(ble_state.device_name);
            
            // Configure advertising data
            esp_ble_adv_data_t adv_data = {
                .set_scan_rsp = false,
                .include_name = true,
                .include_txpower = true,
                .min_interval = 0x0006,
                .max_interval = 0x0010,
                .appearance = 0x00,
                .manufacturer_len = 0,
                .p_manufacturer_data = NULL,
                .service_data_len = 0,
                .p_service_data = NULL,
                .service_uuid_len = 0,
                .p_service_uuid = NULL,
                .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
            };
            esp_ble_gap_config_adv_data(&adv_data);
            break;
        
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(BLE_TAG, "Device connected");
            ble_state.connected = true;
            ble_state.conn_id = param->connect.conn_id;
            break;
        
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(BLE_TAG, "Device disconnected");
            ble_state.connected = false;
            esp_ble_gap_start_advertising(&(esp_ble_adv_params_t){
                .adv_int_min = 0x20,
                .adv_int_max = 0x40,
                .adv_type = ADV_TYPE_IND,
                .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
                .channel_map = ADV_CHNL_ALL,
                .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
            });
            break;
        
        case ESP_GATTS_WRITE_EVT:
            // Handle write to device name characteristic
            if (param->write.len > 0 && param->write.len < BLE_DEVICE_NAME_MAX_LEN) {
                memset(ble_state.device_name, 0, BLE_DEVICE_NAME_MAX_LEN);
                memcpy(ble_state.device_name, param->write.value, param->write.len);
                ESP_LOGI(BLE_TAG, "Device name updated to: %s", ble_state.device_name);
                esp_ble_gap_set_device_name(ble_state.device_name);
            }
            break;
        
        default:
            break;
    }
}

// Initialize Bluetooth
static esp_err_t bluetooth_init(void)
{
    esp_err_t ret;
    
    // Initialize NVS for Bluetooth
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Release memory used by classic BT (ESP32-C3 is BLE only)
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    
    // Initialize Bluetooth controller with ESP32-C3 specific config
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(BLE_TAG, "Bluetooth controller init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(BLE_TAG, "Bluetooth controller enable failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize Bluedroid
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(BLE_TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(BLE_TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Get BLE address
    uint8_t ble_addr[6];
    esp_read_mac(ble_addr, ESP_MAC_BT);
    memcpy(ble_state.ble_addr, ble_addr, 6);
    ESP_LOGI(BLE_TAG, "BLE Address: %02X:%02X:%02X:%02X:%02X:%02X",
             ble_state.ble_addr[0], ble_state.ble_addr[1], ble_state.ble_addr[2],
             ble_state.ble_addr[3], ble_state.ble_addr[4], ble_state.ble_addr[5]);
    
    // Set default device name
    strncpy(ble_state.device_name, DEFAULT_DEVICE_NAME, BLE_DEVICE_NAME_MAX_LEN - 1);
    
    // Register callbacks
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_register_callback(gatts_event_handler);
    
    // Register GATT application
    esp_ble_gatts_app_register(0);
    
    ESP_LOGI(BLE_TAG, "Bluetooth initialized successfully");
    ESP_LOGI(BLE_TAG, "Device name: %s", ble_state.device_name);
    
    return ESP_OK;
}

// Update device name (can be called from console or other interface)
void update_device_name(const char *new_name)
{
    if (new_name && strlen(new_name) < BLE_DEVICE_NAME_MAX_LEN) {
        strncpy(ble_state.device_name, new_name, BLE_DEVICE_NAME_MAX_LEN - 1);
        ble_state.device_name[BLE_DEVICE_NAME_MAX_LEN - 1] = '\0';
        esp_ble_gap_set_device_name(ble_state.device_name);
        ESP_LOGI(BLE_TAG, "Device name updated to: %s", ble_state.device_name);
    }
}

// Get current device name
const char* get_device_name(void)
{
    return ble_state.device_name;
}

// Get BLE address
void get_ble_address(uint8_t *addr)
{
    if (addr) {
        memcpy(addr, ble_state.ble_addr, 6);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting LSM6DSOX Pedometer with BLE on ESP32-C3");
    
    // Initialize motor GPIO
    gpio_reset_pin(MOTOR_GPIO);
    gpio_set_direction(MOTOR_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(MOTOR_GPIO, 0);
    
    // Initialize I2C
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully on GPIO%d (SDA) and GPIO%d (SCL)", 
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    
    // Initialize LSM6DSOX
    ESP_ERROR_CHECK(lsm6dsox_init());
    
    // Initialize Bluetooth
    ESP_ERROR_CHECK(bluetooth_init());
    
    printf("\n=== ESP32-C3 LSM6DSOX Pedometer with BLE ===\n");
    printf("I2C Pins: SDA=GPIO%d, SCL=GPIO%d\n", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    printf("Motor Pin: GPIO%d\n", MOTOR_GPIO);
    printf("Step threshold: %.0f mg\n", STEP_THRESHOLD);
    printf("Stride length: %.2f m\n", AVG_STRIDE_LENGTH_M);
    printf("BLE Device Name: %s\n", ble_state.device_name);
    printf("BLE Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
           ble_state.ble_addr[0], ble_state.ble_addr[1], ble_state.ble_addr[2],
           ble_state.ble_addr[3], ble_state.ble_addr[4], ble_state.ble_addr[5]);
    printf("============================================\n\n");
    
    // Main loop
    while (1) {
        process_accelerometer();
        vTaskDelay(100 / portTICK_PERIOD_MS);  // Update every 100ms for better step detection
    }
}
