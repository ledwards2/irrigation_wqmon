idf_component_register(SRCS "lwjson.c" "lwjson_stream.c" "lwjson_debug.c" "mqtt_comms.c"
                    INCLUDE_DIRS "include"
                    REQUIRES "freertos" "esp_wifi" "esp_event" "esp_system" "nvs_flash" "lwip" "mqtt" "app_update")
