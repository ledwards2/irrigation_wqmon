#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "mqtt_client.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "esp_ota_ops.h"
#include "esp_flash_partitions.h"


#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define WIFI_MAX_RETRY 10
#define EXAMPLE_H2E_IDENTIFIER ""
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK

//#define WIFI_SSID "WiFi-3B5C27"
//#define WIFI_PWD "36498369"

#define WIFI_SSID "HUAWEI P30"
#define WIFI_PWD "b5747801dcbe"

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

extern esp_err_t tago_subscribe(char* topic);

void wqmon_ota_update(char* data, int data_len);
void tago_mqtt_data_cb(esp_mqtt_event_handle_t event);