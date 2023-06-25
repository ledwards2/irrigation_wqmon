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

#define MQTT_BROKER_URL "mqtt://mqtt.tago.io"
#define MQTT_BROKER_PWD "54be2595-549f-44c2-9541-3bb1e362722a"
#define MQTT_BROKER_USERNAME "Token"
// WPA standard max length for passwords is 63 bits 
#define WIFI_PWD_LEN 64 
// WPA standard max length for ssids is 32 bits
#define WIFI_SSID_LEN 33
struct tago_msg {
    char variable[20];
    char unit[20];
    float value; 
};

// name(str), value(float), unit(str)
extern void wifi_init(char ssid[], char pwd[]);
extern void mqtt_init(); 

extern QueueHandle_t mqtt_tx_msgq; 

extern esp_err_t tago_send(struct tago_msg);

extern esp_err_t tago_subscribe(char* topic);
