#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_event.h"
#define WIFI_SSID "WiFi-3B5C27"
#define WIFI_PWD "36498369"

#define MQTT_BROKER_URL "mqtt://mqtt.tago.io"
#define MQTT_BROKER_PWD "54be2595-549f-44c2-9541-3bb1e362722a"
#define MQTT_BROKER_USERNAME "Token"

struct tago_msg {
    char variable[20];
    char unit[20];
    float value; 
};

// name(str), value(float), unit(str)
extern void wifi_init();
extern void mqtt_init(); 

extern QueueHandle_t mqtt_tx_msgq; 

extern esp_err_t tago_send(struct tago_msg);