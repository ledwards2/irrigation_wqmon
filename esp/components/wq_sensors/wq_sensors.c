#include "wq_sensors.h"
esp_err_t analog_sensors_init() {
    esp_err_t err; 
    err = adc1_config_width(ADC_WIDTH_BIT_12);
    if (err != ESP_OK) {
        printf("Could not set width for ADC1\n");
    }

    // ADC1 channel 0 - GPIO36(?), ProS3 Pin 1
    err = adc1_config_channel_atten(EC_ADC_CHANNEL, EC_ADC_ATTEN_DB);
    
    if (err != ESP_OK) {
        printf("Could not set attenuation for EC\n");
    }

    err = adc1_config_channel_atten(PH_ADC_CHANNEL, PH_ADC_ATTEN_DB);
    if (err != ESP_OK) {
        printf("Could not set attenuation fopr pH");
    }

    return err; 
}

float adc_to_msiemen_cm(int adc) {
    float reading = ((float) adc) * 0.005673439 + 0.114762141;
    return reading; 
}

int ec_get_adc() {
    return adc1_get_raw(EC_ADC_CHANNEL);
}

int ph_get_adc() {
    return adc1_get_raw(PH_ADC_CHANNEL);
}
