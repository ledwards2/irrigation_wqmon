/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/adc.h"

float adc_to_msiemen_cm(int adc);

void app_main(void)
{
    printf("Hello world!\n");

    /* Print chip information */
    /*
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());
    */
    esp_err_t err; 
    err = adc1_config_width(ADC_WIDTH_BIT_12);
    if (err != ESP_OK) {
        printf("Could not set width\n");
    }

    // ADC1 channel 0 - GPIO36(?), ProS3 Pin 1
    err = adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
    
    if (err != ESP_OK) {
        printf("Could not set attenuation\n");
    }

    int reading;
    float conductivity; 
    while (1)  {
        reading = adc1_get_raw(ADC1_CHANNEL_0); 
        conductivity = adc_to_msiemen_cm(reading); 
        printf("Reading: %fmS/cm\n", conductivity); 
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

}

float adc_to_msiemen_cm(int adc) {
    float reading = ((float) adc) * 0.005673439 + 0.114762141;
    return reading; 
}