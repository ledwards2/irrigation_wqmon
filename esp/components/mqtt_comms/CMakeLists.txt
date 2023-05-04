idf_component_register(SRCS "mqtt_comms.c"
                    INCLUDE_DIRS "include"
                    REQUIRES "freertos" "esp_wifi" "esp_event" "esp_system" "nvs_flash" "lwip" "mqtt")
