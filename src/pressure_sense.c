#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define HX711_DOUT_PIN GPIO_NUM_1
#define HX711_SCK_PIN  GPIO_NUM_0

static const char *TAG = "HX711";

// Read a single bit from HX711
static uint8_t hx711_shiftIn(void) {
    uint8_t value = 0;
    
    gpio_set_level(HX711_SCK_PIN, 1);
    esp_rom_delay_us(1);
    value = gpio_get_level(HX711_DOUT_PIN);
    gpio_set_level(HX711_SCK_PIN, 0);
    esp_rom_delay_us(1);
    
    return value;
}

// Read raw value from HX711
static int32_t hx711_read(void) {
    int32_t count = 0;
    
    // Wait for HX711 to be ready
    while (gpio_get_level(HX711_DOUT_PIN) == 1) {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    
    // Read 24 bits
    for (int i = 0; i < 24; i++) {
        count = count << 1;
        count |= hx711_shiftIn();
    }
    
    // Set gain for next reading (25 pulses = gain 128, channel A)
    gpio_set_level(HX711_SCK_PIN, 1);
    esp_rom_delay_us(1);
    gpio_set_level(HX711_SCK_PIN, 0);
    esp_rom_delay_us(1);
    
    // Convert to signed 32-bit
    if (count & 0x800000) {
        count |= 0xFF000000;
    }
    
    return count;
}

// Tare (zero) the scale
static int32_t hx711_tare(int samples) {
    int64_t sum = 0;
    
    for (int i = 0; i < samples; i++) {
        sum += hx711_read();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    
    return (int32_t)(sum / samples);
}

// Get average reading
static int32_t hx711_get_value(int samples) {
    int64_t sum = 0;
    
    for (int i = 0; i < samples; i++) {
        sum += hx711_read();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    
    return (int32_t)(sum / samples);
}

void app_main(void) {
    // Configure GPIO pins
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << HX711_SCK_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    io_conf.pin_bit_mask = (1ULL << HX711_DOUT_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);
    
    gpio_set_level(HX711_SCK_PIN, 0);
    
    ESP_LOGI(TAG, "HX711 Load Sensor Test");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    ESP_LOGI(TAG, "DOUT pin state: %d", gpio_get_level(HX711_DOUT_PIN));
    vTaskDelay(100 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Testing HX711 communication...");

    for (int i = 0; i < 5; i++) {
        int32_t test_read = hx711_read();
        ESP_LOGI(TAG, "Test read %d: %ld", i, test_read);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    // Check if HX711 is ready
    if (gpio_get_level(HX711_DOUT_PIN) == 0) {
        ESP_LOGI(TAG, "HX711 found!");
    } else {
        ESP_LOGE(TAG, "HX711 not ready, check wiring");
    }
    
    // Tare the scale
    ESP_LOGI(TAG, "Taring... remove any weight");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    int32_t offset = hx711_tare(10);
    ESP_LOGI(TAG, "Tare offset: %ld", offset);
    
    // Calibration factor - adjust this based on your load cell
    float calibration_factor = 10.0;
    
    ESP_LOGI(TAG, "Starting measurements...");
    ESP_LOGI(TAG, "Place known weight and adjust calibration_factor in code");
    
    while (1) {
//        int32_t raw_value = hx711_get_value(10);
//        int32_t adjusted_value = raw_value - offset;
        
        // Convert to weight units
       // float weight_grams = adjusted_value / calibration_factor;
       // float weight_kg = weight_grams / 1000.0;
       // float weight_newtons = weight_kg * 9.81;
       // float weight_lbs = weight_grams * 0.00220462;
        
       // ESP_LOGI(TAG, "Raw: %ld | %.1f g | %.3f kg | %.2f N | %.2f lbs", 
         //        adjusted_value, weight_grams, weight_kg, weight_newtons, weight_lbs);
        
       // vTaskDelay(500 / portTICK_PERIOD_MS);
	int32_t raw_value = hx711_read();  // Single raw read, no averaging
   
	ESP_LOGI(TAG, "Raw reading: %ld", raw_value);
    
    	vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
