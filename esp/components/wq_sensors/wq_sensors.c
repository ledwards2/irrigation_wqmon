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
        printf("Could not set attenuation for EC\r\n");
    }

    err = adc1_config_channel_atten(PH_ADC_CHANNEL, PH_ADC_ATTEN_DB);
    if (err != ESP_OK) {
        printf("Could not set attenuation for pH\r\n");
    }

    err = adc1_config_channel_atten(TDS_ADC_CHANNEL, TDS_ADC_ATTEN_DB);
    if (err != ESP_OK) {
        printf("Could not set attenuation for TDS\r\n");
    }

    err = adc1_config_channel_atten(TURB_ADC_CHANNEL, TURB_ADC_ATTEN_DB);
    if (err != ESP_OK) {
        printf("Could not set attenuation for turbidity\r\n");
    }
    
    return err; 
}

float adc_to_msiemen_cm(int adc) {
    //float reading = ((float) adc) * 0.005673439 + 0.114762141;
    return ((float) (adc)) * ADC_EC_SLOPE + ADC_EC_INTERCEPT;
    // return reading; 
}

float adc_to_ph(int adc) {
    return ((float) (adc)) * ADC_PH_SLOPE + ADC_PH_INTERCEPT;
}

float adc_to_turb(int adc) {
    return ((float) (adc)) * ADC_TURB_SLOPE + ADC_TURB_INTERCEPT;
}

float adc_to_tds(int adc) {
    return ((float) (adc)) * ADC_TDS_SLOPE + ADC_TDS_INTERCEPT;
}

int ec_get_adc() {
    return adc1_get_raw(EC_ADC_CHANNEL);
}

int ph_get_adc() {
    return adc1_get_raw(PH_ADC_CHANNEL);
}

int turb_get_adc() {
    return adc1_get_raw(TURB_ADC_CHANNEL); 
}

int tds_get_adc() {
    return adc1_get_raw(TDS_ADC_CHANNEL);
}
