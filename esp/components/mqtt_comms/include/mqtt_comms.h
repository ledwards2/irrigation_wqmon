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
#include <string.h>
//#include "esp_ota_ops.h"
//#include "esp_flash_partitions.h"


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

#define MQTT_RX_EVENT_QUEUE_LEN 10 
#define RX_EVENT_SEND_WAIT 1
#define TAGO_MSG_BUF_INIT_SIZE 128

struct tago_msg {
    char variable[20];
    char unit[20];
    float value; 
};

// name(str), value(float), unit(str)
extern void wifi_init(char ssid[], char pwd[]);
extern void mqtt_init(); 
extern EventGroupHandle_t s_wifi_event_group;

extern QueueHandle_t mqtt_tx_msgq; 

extern esp_err_t tago_send(struct tago_msg);

extern esp_err_t tago_subscribe(char* topic);
void received_event_handler(void* _unused);



#define MQTT_MSG_TOPIC_LEN 63
#define MQTT_MSG_DATA_LEN 127
#define CALI_QUEUE_LEN 1 
#define NOTIFICATION_SENSOR_LEN 20

struct CalibrationNotification {
    char variable[NOTIFICATION_SENSOR_LEN]; 
    float value;
};

extern QueueHandle_t CalibrationValues;

