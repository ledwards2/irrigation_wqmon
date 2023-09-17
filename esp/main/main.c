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
#include <driver/i2c.h> 
#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"
#include "nau7802.h"

#define GPIO_DS18B20_0       (12)
#define DS18B20_MAX_DEVICES          (1)
#define DS18B20_RESOLUTION   (DS18B20_RESOLUTION_12_BIT)
#define DS18B20_SAMPLE_PERIOD        (1000)   // milliseconds


#define SENSOR_POLL_TIME_MS 60 * 1000 * 2
#define I2C_SCL 9
#define I2C_SDA 10 
#define I2C_PORT I2C_NUM_1
void ds18b20_handler(void* pvParam);
void handler_thread(void* _unused);
void adc_sensor_handler(void* pvParam);
void spiffs_read_cal_param(char* fileName, float* val); 
void spiffs_write_cal_param(char* fileName, float value);

void nau7802_handler(void* pvParam);
QueueHandle_t sensorMessages;

EventGroupHandle_t calibrationFlags; 
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

    mount_spiffs();
    char pwd[WIFI_PWD_LEN]; 
    char ssid[WIFI_SSID_LEN];
    spiffs_read_creds(ssid, pwd);

    
    wifi_init(ssid, pwd);

    uint8_t mac[6]; 
    esp_wifi_get_mac(ESP_IF_WIFI_STA, mac); 
    ESP_LOGI("MAC address", "MAC address: %02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    mqtt_init();

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    tago_subscribe("tago/down");
    
    ghota_config_t ghconfig = {
        .filenamematch = "irrigation_wqmon_esp32s3.bin",
        //.orgname = "ledwards2", 
        //.reponame = "irrigation_wqmon",
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
    ESP_ERROR_CHECK(ghota_start_update_task(ghota_client));

    // Do this to poll for updates based on ghota_config_t.updateInterval
    ESP_ERROR_CHECK(ghota_start_update_timer(ghota_client));
    sensorMessages = xQueueCreate(10, sizeof(struct tago_msg));


    calibrationFlags = xEventGroupCreate();  
    void* param = &sensorMessages; 
    xTaskCreate(
        ds18b20_handler, 
        "ds18b20",
        3 * configMINIMAL_STACK_SIZE, 
        &param,
        tskIDLE_PRIORITY + 3, 
        NULL);
    
    xTaskCreate(
        adc_sensor_handler, 
        "adc_sensor",
        5 * configMINIMAL_STACK_SIZE,
        &param,
        tskIDLE_PRIORITY + 3, 
        NULL); 
    
    xTaskCreate(
        nau7802_handler, 
        "nau7802", 
        5 * configMINIMAL_STACK_SIZE, 
        &param, 
        tskIDLE_PRIORITY + 3, 
        NULL); 
    
    
    struct tago_msg msgRx;
    

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

void adc_sensor_handler(void* pvParam) { 
    AdcSensorConfig_t senseCfg; 

    spiffs_read_cal_param("/spiffs/ec_slope.txt", &senseCfg.ecSlope); 
    spiffs_read_cal_param("/spiffs/ec_intercept.txt", &senseCfg.ecInt); 
    spiffs_read_cal_param("/spiffs/ph_slope.txt", &senseCfg.phSlope); 
    spiffs_read_cal_param("/spiffs/ph_intercept.txt", &senseCfg.phInt); 
    spiffs_read_cal_param("/spiffs/tds_slope.txt", &senseCfg.tdsSlope); 
    spiffs_read_cal_param("/spiffs/tds_intercept.txt", &senseCfg.tdsInt);
    spiffs_read_cal_param("/spiffs/turb_slope.txt", &senseCfg.turbSlope); 
    spiffs_read_cal_param("/spiffs/turb_intercept.txt", &senseCfg.turbInt);
    ESP_LOGE(TAG, "startup Ec slope: %f", senseCfg.ecSlope);
    analog_sensors_init(&senseCfg);
    


    
    int ecReading;
    int phReading;
    int turbReading;
    int tdsReading; 
    float conductivity;  

    struct tago_msg ecMsg, phMsg, turbMsg, tdsMsg; 
    memcpy(&ecMsg.unit, "mS/cm", 6);
    memcpy(&ecMsg.variable, "EC", 3);

    memcpy(phMsg.unit, "pH", 3);
    memcpy(phMsg.variable, "pH", 3); 

    memcpy(turbMsg.unit, "turb", 5); 
    memcpy(turbMsg.variable, "Turbidity", sizeof("Turbidity")); 

    memcpy(tdsMsg.unit, "%", 2); 
    memcpy(tdsMsg.variable, "TDS", 4); 

    const char* TAG = "adc_sensor_handler"; 

    while (!sensorMessages) {
        vTaskDelay(100); 
    }
    EventBits_t flags; 
    int calStatus; 
    struct CalibrationNotification notification; 
    BaseType_t msgPending; 
    
    while (1) {
        // if (calibration flag) then 
        
        // get calibration value 
        // set calibration value 
        // Peek the queue, check if it is a sensor this thread cares about 
        msgPending = xQueuePeek(CalibrationValues, &notification, 10); 
        if (msgPending == pdTRUE) {
            ESP_LOGI(TAG, "Pending message in ADC queue");
            calStatus = set_cal_if_adc_sensor(notification.variable, notification.value, &senseCfg);
            if (calStatus != CALIBRATE_FAIL) {
                // We had a message. It was for one of these sensors. Take it out of the queue.
                xQueueReceive(CalibrationValues, &notification, 10); 
                
                // Now save to SPIFFS 
                if (calStatus != CALIBRATE_NO_CHANGE) {
                    char* slopeFileName; 
                    char* intFileName; 
                    float slope; 
                    float intercept; 

                    if (calStatus == CALIBRATED_EC) {
                        slopeFileName = "/spiffs/ec_slope.txt"; 
                        intFileName = "/spiffs/ec_intercept.txt"; 
                        slope = senseCfg.ecSlope; 
                        intercept = senseCfg.ecInt; 
                    } else if (calStatus == CALIBRATED_PH) {
                        slopeFileName = "/spiffs/ph_slope.txt"; 
                        intFileName = "/spiffs/ph_intercept.txt"; 
                        slope = senseCfg.phSlope; 
                        intercept = senseCfg.phInt; 
                    } else if (calStatus == CALIBRATED_TURB) {
                        slopeFileName = "/spiffs/turb_slope.txt"; 
                        intFileName = "/spiffs/turb_intercept.txt"; 
                        slope = senseCfg.turbSlope; 
                        intercept = senseCfg.turbInt; 
                    } else if (calStatus == CALIBRATED_TDS) {
                        slopeFileName = "/spiffs/tds_slope.txt"; 
                        intFileName = "/spiffs/tds_intercept.txt";        
                        slope = senseCfg.tdsSlope; 
                        intercept = senseCfg.tdsInt; 
                    } else {
                        ESP_LOGE(TAG, "Calibration result not known"); 
                        continue; 
                    }
                    ESP_LOGI(TAG, "Writing slope: %f for, %s, int %f to %s", slope, slopeFileName, intercept, intFileName);
                    spiffs_write_cal_param(slopeFileName, slope); 
                    spiffs_write_cal_param(intFileName, intercept); 

                    float read; 
                    spiffs_read_cal_param("/spiffs/ec_slope.txt", &read); 
                    ESP_LOGI(TAG, "Read EC slope as %f", read); 

                } 
                
            } else {
                ESP_LOGI(TAG, "Calibration not ADC"); 
            }
        } 

        ecReading = ec_get_adc(); 
        phReading = ph_get_adc(); 
        turbReading = turb_get_adc(); 
        tdsReading = tds_get_adc(); 
        ESP_LOGI("ADC: ", "ec: %i, ph: %i, turb: %i, tds: %i", 
                ecReading, phReading, turbReading, tdsReading);
        
        ecMsg.value = adc_to_msiemen_cm(ecReading, &senseCfg); 

        phMsg.value = adc_to_ph(phReading, &senseCfg); 
        turbMsg.value = adc_to_turb(turbReading, &senseCfg); 
        tdsMsg.value = adc_to_tds(tdsReading, &senseCfg); 
        ESP_LOGI(TAG, "pH Slope: %f, int: %f", senseCfg.phSlope, senseCfg.phInt);

        ESP_LOGI(TAG, "ec Slope: %f, int: %f", senseCfg.ecSlope, senseCfg.ecInt);

        if (xQueueSendToBack(sensorMessages, &ecMsg, 10) != pdTRUE) {
            ESP_LOGE(TAG, "Could not send to handler"); 
        }

        if (xQueueSendToBack(sensorMessages, &phMsg, 10) != pdTRUE) {
            ESP_LOGE(TAG, "Could not send to handler"); 
        }

        if (xQueueSendToBack(sensorMessages, &turbMsg, 10) != pdTRUE) {
            ESP_LOGE(TAG, "Could not send to handler"); 
        }

        if (xQueueSendToBack(sensorMessages, &tdsMsg, 10) != pdTRUE) {
            ESP_LOGE(TAG, "Could not send to handler"); 
        }

        // vTaskDelay(SENSOR_POLL_TIME_MS / portTICK_PERIOD_MS);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }


}
void spiffs_write_cal_param(char* fileName, float value) {
    FILE* f = fopen(fileName, "w");
    if (f != NULL) {
        fprintf(f, "%f", value); 
    } else {
        ESP_LOGW("spiffs-write-cal-param", "file %s for writing not found", fileName);
    }
    fclose(f); 

}

void spiffs_read_cal_param(char* fileName, float* valueRead) {
    FILE* f = fopen(fileName, "r"); 
    if (f != NULL) {
        fscanf(f, "%f", valueRead); 
    } else {
        ESP_LOGW("spiffs-read-cal-param", "file %s for reading not found", fileName); 
        *valueRead = 0;
    }
    fclose(f); 

}


void nau7802_handler(void* pvParam) {
    struct nau7802_handle handle; 
    struct tago_msg strainMsg; 
    struct tago_msg weightMsg; 
    memcpy(strainMsg.variable, "strain", sizeof("strain")); 
    memcpy(strainMsg.unit, "raw", sizeof("raw")); 
    
    memcpy(weightMsg.variable, "weight", sizeof("weigt")); 
    memcpy(weightMsg.unit, "kg", sizeof("kg")); 

    esp_err_t res = ESP_FAIL;
    while (res != ESP_OK) {
        ESP_LOGI("nau7802", "trying to initialize"); 
        res = nau7802_init(I2C_SDA, I2C_SCL, I2C_PORT, &handle); 

        vTaskDelay(500 / portTICK_PERIOD_MS); 
    }
    
    // Don't start working until the queue is ready 
    while (!sensorMessages) {
        vTaskDelay(100); 
    }
    struct CalibrationNotification notification; 


    BaseType_t msgPending; 
    NAU7802CalConfig_t strainCfg; 
    strainCfg.handle = &handle;
    spiffs_read_cal_param("/spiffs/strain_slope.txt", &strainCfg.strainSlope); 
    spiffs_read_cal_param("/spiffs/strain_intercept.txt", &strainCfg.strainInt); 
    int32_t strain; 
    int calStatus; 
    while (1) {

        msgPending = xQueuePeek(CalibrationValues, &notification, 10); 
        if (msgPending == pdTRUE) {
            ESP_LOGE(TAG, "Got from queue!");
            calStatus = set_cal_if_strain(notification.variable, notification.value, &strainCfg);
            ESP_LOGE(TAG, "Set if strain, %i", calStatus);
            if (calStatus != CALIBRATE_STRAIN_FAIL) {
                // We had a message. It was for one of these sensors. Take it out of the queue.
                xQueueReceive(CalibrationValues, &notification, 10); 
                ESP_LOGI(TAG, "Got from queue");
                // Now save to SPIFFS 
                if (calStatus != CALIBRATE_STRAIN_NO_CHANGE) {
                    char* slopeFileName; 
                    char* intFileName; 
                    float slope; 
                    float intercept; 

                    if (calStatus == CALIBRATED_STRAIN) {
                        slopeFileName = "/spiffs/strain_slope.txt"; 
                        intFileName = "/spiffs/strain_intercept.txt"; 
                        slope = strainCfg.strainSlope; 
                        intercept = strainCfg.strainInt; 
                    } else {
                        ESP_LOGE(TAG, "Calibration result not known"); 
                        continue; 
                    }
                    ESP_LOGI(TAG, "Writing slope: %f for, %s, int %f to %s", slope, slopeFileName, intercept, intFileName);
                    spiffs_write_cal_param(slopeFileName, slope); 
                    spiffs_write_cal_param(intFileName, intercept); 
                } 
                
            } else {
                ESP_LOGI(TAG, "Calibration not strain"); 
            }
        } 


        ESP_LOGI(TAG, "Reading strain");
        strain = nau7802_read_conversion(&handle); 
        strainMsg.value = strain; 

        weightMsg.value = nau7802_get_weight(strain, &strainCfg);
        if (xQueueSendToBack(sensorMessages, &strainMsg, 10) != pdTRUE) {
            ESP_LOGE(TAG, "Could not send to handler"); 
        }

        if (xQueueSendToBack(sensorMessages, &weightMsg, 10) != pdTRUE) {
            ESP_LOGE(TAG, "Could not send to handler"); 
        }
        ESP_LOGI("nau7802", "strain: %li", strain);
        ESP_LOGI("Weight: " , "%f", weightMsg.value);
        vTaskDelay(SENSOR_POLL_TIME_MS / portTICK_PERIOD_MS);
    }
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
    while (!found) {
        owb_search_first(owb, &search_state, &found);
        owb_search_first(owb, &search_state, &found);
        if (!found) { 
            ESP_LOGE(TAG, "DS18B20 Not Found");
            vTaskDelay(5000 / portTICK_PERIOD_MS);
        }
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
        vTaskDelay(SENSOR_POLL_TIME_MS / portTICK_PERIOD_MS); 
    }
}