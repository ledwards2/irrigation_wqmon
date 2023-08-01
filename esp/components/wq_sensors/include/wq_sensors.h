
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include "driver/adc.h"
#define EC_ADC_WIDTH ADC_WIDTH_BIT_12



#define PH_ADC_CHANNEL ADC1_CHANNEL_3
#define EC_ADC_CHANNEL ADC1_CHANNEL_4
#define TDS_ADC_CHANNEL ADC1_CHANNEL_5
#define TURB_ADC_CHANNEL ADC1_CHANNEL_6

#define EC_ADC_ATTEN_DB ADC_ATTEN_DB_11
#define PH_ADC_ATTEN_DB ADC_ATTEN_DB_11
#define TDS_ADC_ATTEN_DB ADC_ATTEN_DB_11
#define TURB_ADC_ATTEN_DB ADC_ATTEN_DB_11

#define ADC_EC_SLOPE 0.005673439
#define ADC_EC_INTERCEPT 0.114762141

#define ADC_PH_SLOPE 1
#define ADC_PH_INTERCEPT 0 

#define ADC_TURB_SLOPE 1
#define ADC_TURB_INTERCEPT 0 

#define ADC_TDS_SLOPE 1
#define ADC_TDS_INTERCEPT 0 



extern float adc_to_msiemen_cm(int adc);
extern float adc_to_ph(int adc); 
extern float adc_to_turb(int adc); 
extern float adc_to_tds(int adc); 
extern int ec_get_adc();
extern int ph_get_adc(); 
extern int turb_get_adc(); 
extern int tds_get_adc(); 
extern esp_err_t analog_sensors_init();


