#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_spiffs.h"
#include "esp_event.h"
//#include "esp_system.h"
//#include "esp_spi_flash.h"

#include "wq_sensors.h"
#include "mqtt_comms.h"

#include <string.h>
#include "lwip/err.h"
#include <lwip/sys.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_ghota.h>

#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"

#define GPIO_DS18B20_0       (12)
#define DS18B20_MAX_DEVICES          (1)
#define DS18B20_RESOLUTION   (DS18B20_RESOLUTION_12_BIT)
#define DS18B20_SAMPLE_PERIOD        (1000)   // milliseconds
void ds18b20_handler(void* pvParam);
void handler_thread(void* _unused);
QueueHandle_t sensorMessages;
int spiffs_read_creds(char ssid[], char pwd[]);
/* For the ESP-IDF logging API */
static const char* TAG = "main";
void unmount_spiffs() {
    esp_vfs_spiffs_unregister("storage");
}

static void read_from_spiffs(void)
{
    ESP_LOGI(TAG, "Reading hello.txt");

    // Open for reading hello.txt
    FILE* f = fopen("/spiffs/test.txt", "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open test.txt");
        return;
    }

    char buf[64];
    memset(buf, 0, sizeof(buf));
    fread(buf, 1, sizeof(buf), f);
    fclose(f);

    // Display the read contents from the file
    ESP_LOGI(TAG, "Read from test.txt: %s", buf);
}

int spiffs_read_creds(char ssid[], char pwd[]) {
    FILE* ssid_spiffs = fopen("/spiffs/ssid.txt", "r"); 
    FILE* pwd_spiffs = fopen("/spiffs/pwd.txt", "r"); 

    if (ssid_spiffs == NULL || pwd_spiffs == NULL) {
        ESP_LOGE(TAG, "Couldn't retrieve wifi credentials from storage");
        return 0; 
    }

    memset(ssid, 0, WIFI_SSID_LEN); 
    memset(pwd, 0, WIFI_PWD_LEN);
    //fread(ssid, 1, WIFI_SSID_LEN, ssid_spiffs); 
    //fread(pwd, 1, WIFI_PWD_LEN, pwd_spiffs); 
    fgets(ssid, WIFI_SSID_LEN, ssid_spiffs);
    fgets(pwd, WIFI_PWD_LEN, pwd_spiffs);
    ESP_LOGI(TAG, "Got from spiffs: %s, %s", ssid, pwd); 

    fclose(ssid_spiffs); 
    fclose(pwd_spiffs); 
    return 1;
}

void mount_spiffs() {
    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = "storage",
      .max_files = 5,
      .format_if_mount_failed = false
    };

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info("storage", &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
    read_from_spiffs();
}

static void ghota_event_callback(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    ghota_client_handle_t *client = (ghota_client_handle_t *)handler_args;
    ESP_LOGI(TAG, "Got Update Callback: %s", ghota_get_event_str(id));
    if (id == GHOTA_EVENT_START_STORAGE_UPDATE) {
        ESP_LOGI(TAG, "Starting storage update");
        /* if we are updating the SPIFF storage we should unmount it */
        unmount_spiffs();
    } else if (id == GHOTA_EVENT_FINISH_STORAGE_UPDATE) {
        ESP_LOGI(TAG, "Ending storage update");
        /* after updating we can remount, but typically the device will reboot shortly after recieving this event. */
        mount_spiffs();
    } else if (id == GHOTA_EVENT_FIRMWARE_UPDATE_PROGRESS) {
        /* display some progress with the firmware update */
        ESP_LOGI(TAG, "Firmware Update Progress: %d%%", *((int*) event_data));
    } else if (id == GHOTA_EVENT_STORAGE_UPDATE_PROGRESS) {
        /* display some progress with the spiffs partition update */
        ESP_LOGI(TAG, "Storage Update Progress: %d%%", *((int*) event_data));
    }
    (void)client;
    return;
}

void app_main(void) {
    xTaskCreate(handler_thread, 
            "handler", 
            100 * configMINIMAL_STACK_SIZE, 
            NULL,
            tskIDLE_PRIORITY + 1, 
            NULL);
}
void handler_thread(void* _unused)
{
    printf("Hello world!\n");

    mount_spiffs();
    char pwd[WIFI_PWD_LEN]; 
    char ssid[WIFI_SSID_LEN];
    spiffs_read_creds(ssid, pwd);
    ESP_LOGI(TAG, "here: %s, %s", pwd, ssid);

    
    wifi_init(ssid, pwd);
    
    
    vTaskDelay(1000);
    mqtt_init();

    vTaskDelay(1000);
    //tago_subscribe("wqmon/firmware/rx");
    
    ghota_config_t ghconfig = {
        .filenamematch = "irrigation_wqmon_esp32s3.bin",
        // Don't OTA update storage partition. 
        .storagenamematch = "-",
        .storagepartitionname = "storage", 
        .updateInterval = 30, 
    }; 
    
    ghota_client_handle_t* ghota_client = ghota_init(&ghconfig); 

    if (!ghota_client) {
        ESP_LOGE(TAG, "ghota_client_init failed"); 
        return; 
    } else {
        ESP_LOGD(TAG, "ghota init success");
    }
    esp_event_handler_register(GHOTA_EVENTS, ESP_EVENT_ANY_ID, &ghota_event_callback, ghota_client);


    // Do this to try to update immediately
    //ESP_ERROR_CHECK(ghota_start_update_task(ghota_client));

    // Do this to poll for updates based on ghota_config_t.updateInterval
    ESP_ERROR_CHECK(ghota_start_update_timer(ghota_client));
    sensorMessages = xQueueCreate(10, sizeof(struct tago_msg)); 
    void* param = &sensorMessages; 
    xTaskCreate(
        ds18b20_handler, 
        "ds18b20",
        3 * configMINIMAL_STACK_SIZE, 
        &param,
        tskIDLE_PRIORITY + 3, 
        NULL);
    


    int reading;
    float conductivity; 
    struct tago_msg msgRx;
    struct tago_msg msg; 
    memcpy(&msg.unit, "mS", 3);
    memcpy(&msg.variable, "EC", 3);

    struct tago_msg temp_msg;
    memcpy(temp_msg.unit, "deg C", 6);
    memcpy(&temp_msg.variable, "temp", 5); 

    while (1)  {
        /*
        reading = adc1_get_raw(ADC1_CHANNEL_0); 
        conductivity = adc_to_msiemen_cm(reading); 
        printf("Reading: %fmS/cm\n", conductivity); 
        msg.value = conductivity;
        tago_send(msg);


        tago_send(temp_msg);
        */ 

       if (xQueueReceive(sensorMessages, &msgRx, portMAX_DELAY)) {
        tago_send(msgRx); 
       }
        //vTaskDelay(10000 / portTICK_PERIOD_MS);

    }

}

void adc_sensor_handler(QueueHandle_t sendTo) { 
    analog_sensors_init(); 

}


void ds18b20_handler(void* pvParam) {
    QueueHandle_t sendTo = *(QueueHandle_t*) pvParam;
    // Create a 1-Wire bus, using the RMT timeslot driver
    OneWireBus * owb;
    owb_rmt_driver_info rmt_driver_info;
    owb = owb_rmt_initialize(&rmt_driver_info, GPIO_DS18B20_0, RMT_CHANNEL_0, RMT_CHANNEL_4);
    owb_use_crc(owb, true);  // enable CRC check for ROM code

    OneWireBus_SearchState search_state = {0};
    bool found = false;
    owb_search_first(owb, &search_state, &found);
    owb_search_first(owb, &search_state, &found);
    if (!found) { 
        ESP_LOGE(TAG, "DS18B20 Not Found");
    }

    DS18B20_Info* dev = ds18b20_malloc(); 
    ds18b20_init_solo(dev, owb); 
    ds18b20_use_crc(dev, true); 
    ds18b20_set_resolution(dev, DS18B20_RESOLUTION); 


    float temp; 
    struct tago_msg msg;
    const char* TAG = "ds18b20_handler"; 
    memcpy(msg.unit, "deg C", 6);
    memcpy(&msg.variable, "temp", 5); 

    // Don't start working until the queue is ready 
    while (!sendTo) {
        vTaskDelay(100); 
    }
    while (1) {
        ds18b20_convert_all(owb);

            // In this application all devices use the same resolution,
            // so use the first device to determine the delay
        ds18b20_wait_for_conversion(dev);

        ds18b20_read_temp(dev, &msg.value); 
        ESP_LOGI(TAG, "Temp: %f", msg.value); 
        if (xQueueSendToBack(sensorMessages, &msg, 10) != pdTRUE) {
            ESP_LOGE(TAG, "Could not send to queue");
        }
        vTaskDelay(10000 / portTICK_PERIOD_MS); 
    }
}