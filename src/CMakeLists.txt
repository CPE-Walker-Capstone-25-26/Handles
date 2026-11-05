idf_component_register(SRCS "hello.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_i2c
                             esp_driver_gpio
                             nvs_flash
                             bt
                             esp_timer
                             esp_event)
