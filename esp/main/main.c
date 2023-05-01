#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/adc.h"

float adc_to_msiemen_cm(int adc);

#define EC_ADC_WIDTH ADC_WIDTH_BIT_12
#define EC_ADC_CHANNEL ADC1_CHANNEL_0
#define EC_ADC_ATTEN_DB ADC_ATTEN_DB_11

esp_err_t ec_sensor_init() {
    esp_err_t err; 
    err = adc1_config_width(ADC_WIDTH_BIT_12);
    if (err != ESP_OK) {
        printf("Could not set width\n");
    }

    // ADC1 channel 0 - GPIO36(?), ProS3 Pin 1
    err = adc1_config_channel_atten(EC_ADC_CHANNEL, EC_ADC_ATTEN_DB);
    
    if (err != ESP_OK) {
        printf("Could not set attenuation\n");
    }

    return err; 
}



void app_main(void)
{
    printf("Hello world!\n");

    ec_sensor_init(); 

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

