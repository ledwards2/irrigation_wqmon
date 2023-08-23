#include <stdio.h>
#include "mqtt_comms.h"
#include "lwjson.h"
esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URL,
        .credentials.username = MQTT_BROKER_USERNAME,
        .credentials.authentication.password = MQTT_BROKER_PWD,
    };

esp_mqtt_client_handle_t client;
static int s_retry_num = 0;
struct MqttMsg {
    char topic[MQTT_MSG_TOPIC_LEN + 1]; 
    char data [MQTT_MSG_DATA_LEN + 1]; 
}; 
static const char *TAG = "wifi station";
QueueHandle_t receivedEvents; 

static EventGroupHandle_t s_wifi_event_group;
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init(char wifi_ssid[], char wifi_pwd[])
{
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }  
    ESP_ERROR_CHECK(ret);
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            // Example code does this - only works with #DEFINES.
            //.ssid = wifi_ssid, 
            //.password = wifi_pwd,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (pasword len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            //.sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            //.sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };

    // Copy in password and SSID from parameters 
    memcpy(wifi_config.sta.ssid, wifi_ssid, WIFI_SSID_LEN);
    memcpy(wifi_config.sta.password, wifi_pwd, WIFI_PWD_LEN);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 wifi_ssid, wifi_pwd);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 wifi_ssid, wifi_pwd);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    
    struct MqttMsg msg; 
    memcpy(msg.topic, "tago/my_topic", sizeof("tago/my_topic")); 
    memcpy(msg.data, "{\"variable\":\"cali_ec_1\",\"value\":\"55\",\"unit\":\"mS/cm\"}", sizeof("{\"variable\":\"cali_ec_1\",\"value\":\"55\",\"unit\":\"mS/cm\"}"));

    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        // Try reconnecting 
        esp_mqtt_client_reconnect(client);
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        // We have received something via MQTT
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);

        if (event->topic_len > MQTT_MSG_TOPIC_LEN || event->data_len > MQTT_MSG_DATA_LEN) {
            ESP_LOGE(TAG, "Received MQTT message is too large");
            break; 
        }
        memcpy(msg.topic, event->topic, event->topic_len); 
        msg.topic[event->topic_len] = '\0'; 

        memcpy(msg.data, event->data, event->data_len); 
        msg.data[event->data_len] = '\0';
        
        ESP_LOGI(TAG, "Sending: %s, %s", msg.topic, msg.data);        
        if (receivedEvents == NULL) {
            // Queue isn't ready yet. 
            break; 
        }
        if (xQueueSendToBack(receivedEvents, &msg, 10) != pdTRUE) {
            ESP_LOGI(TAG, "Data event queue full!"); 
        }
        ESP_LOGI(TAG, "Sent!");
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);

            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

void mqtt_tx_queue_init(void) {
    //mqtt_tx_msgq = xQueueCreate(MQTT_TX_QUEUE_LEN, sizeof(struct tago_msg));
}


void mqtt_init() {
    mqtt_tx_queue_init(); 
    /* 
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("outbox", ESP_LOG_VERBOSE);
    */ 

    receivedEvents = xQueueCreate(MQTT_RX_EVENT_QUEUE_LEN, sizeof(struct MqttMsg));
    if (receivedEvents == NULL) {
        ESP_LOGE(TAG, "Couldn't create received message queue");
    }
    BaseType_t status = xTaskCreate(received_event_handler, "rx event handler", 100 * configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, NULL); 
    if (status != pdPASS) {
        ESP_LOGE(TAG, "Couldn't allocate memory for task");
    }
    ESP_LOGI(TAG, "Initializing MQTT");
    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

QueueHandle_t CalibrationValues; 


void received_event_handler(void* _unused) {
    
    struct MqttMsg msg; 
    static lwjson_token_t tokens[128]; 
    static lwjson_t lwjson; 
    int variableLen; 
    int valueLen; 
    
    char* variable; 
    float value;  
    CalibrationValues = xQueueCreate(CALI_QUEUE_LEN, sizeof(struct CalibrationNotification));
    struct CalibrationNotification notification; 
    lwjson_init(&lwjson, tokens, LWJSON_ARRAYSIZE(tokens));
    while (1) {
        vTaskDelay(69); 
        ESP_LOGE(TAG, "Received event handler :) "); 

        if (xQueueReceive(receivedEvents, &msg, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "Unblocked");
            // We have new data to process. 
            ESP_LOGI(TAG, "Got: %s, %s", msg.topic, msg.data); 
            printf("got: %s, %s", msg.topic, msg.data);
            if (!strcmp(msg.topic, "tago/down")) {    
                
                if (lwjson_parse(&lwjson, msg.data) == lwjsonOK) {
                    ESP_LOGI(TAG, "json parsed");
                    const lwjson_token_t * t; 
                
                    if ((t = lwjson_find(&lwjson, "variable")) != NULL) {
                        ESP_LOGI(TAG, "Variable: %.*s", t->u.str.token_value_len, t->u.str.token_value);
                        variable = t->u.str.token_value; 
                        variableLen = t->u.str.token_value_len; 
                    } else {
                        // Don't bother with the rest of it - this message doesn't have a variable. 
                        continue; 
                    }
                    
                    if ((t = lwjson_find(&lwjson, "value")) != NULL) {
                        if (t->type == LWJSON_TYPE_NUM_REAL || t->type == LWJSON_TYPE_NUM_INT) {
                            value = t->u.num_real; 
                      
                            ESP_LOGI(TAG, "Value: %f", value);
                        } else { 
                            ESP_LOGE(TAG, "Calibration value must be real float or int, was: %i", t->type); 
                            continue; 
                        }
                    } else {
                        continue; 
                    }


                    notification.value = value; 
                    if (NOTIFICATION_SENSOR_LEN < variableLen + 1) {
                        ESP_LOGE(TAG, "Sensor name too long - change length definition"); 
                        continue; 
                    }
                    // Now we have valid variable and lengths. 

                    memcpy(notification.variable, variable, variableLen); 
                    notification.variable[variableLen] = '\0';
                    xQueueSendToBack(CalibrationValues, &notification, 0);  
                }
            }
        } else {
            // If we didn't receive, the queue might not be ready.
            vTaskDelay(10 / portTICK_PERIOD_MS); 
        }
    }
    // If we break the loop, this should get freed. 
     lwjson_free(&lwjson); 
}


char* tago_format_msg(char* variable, float value, char* unit) {
    char* msg = malloc(TAGO_MSG_BUF_INIT_SIZE * sizeof(char));

    sprintf(msg, "{\"variable\": \"%s\", \"value\": %f, \"unit\": \"%s\"}", 
        variable, 
        value, 
        unit
        );
    return msg; 
}

esp_err_t tago_send(struct tago_msg msg) {
    //return xQueueSendToBack(mqtt_tx_msgq, msg);
    char* mqtt_msg = tago_format_msg(msg.variable, msg.value, msg.unit);
    esp_err_t msg_id = esp_mqtt_client_publish(client, "tago/data/post", mqtt_msg, 0, 0, 0);
    free(mqtt_msg);
    return msg_id;
}

esp_err_t tago_subscribe(char* topic) {
    int qos = 1; 
    return (esp_err_t) (esp_mqtt_client_subscribe(client, topic, qos));
}