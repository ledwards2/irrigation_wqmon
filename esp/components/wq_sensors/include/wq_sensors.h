
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include "driver/adc.h"
#include <string.h>
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

#define EC_CAL_1 "cali_ec_1"
#define EC_CAL_2 "cali_ec_2"
#define PH_CAL_1 "cali_ph_1"
#define PH_CAL_2 "cali_ph_2"
#define TURB_CAL_1 "cali_turb_1"
#define TURB_CAL_2 "cali_turb_2"
#define TDS_CAL_1 "cali_tds_1"
#define TDS_CAL_2 "cali_tds_2" 



#define CALIBRATE_FAIL 0 

#define CALIBRATE_NO_CHANGE 1
#define CALIBRATED_EC 2
#define CALIBRATED_PH 3 
#define CALIBRATED_TURB 4
#define CALIBRATED_TDS 5 
int adc_get_avg(adc1_channel_t chan, int numSamples);

struct AdcSensorConfig {
    adc1_channel_t ecChannel; 
    adc_atten_t ecAtten; 
    adc1_channel_t phChannel; 
    adc_atten_t phAtten; 

    adc1_channel_t turbChannel;     
    adc_atten_t turbAtten; 

    adc1_channel_t tdsChannel; 
    adc_atten_t tdsAtten; 

    int knownEcRaw[2]; 
    float knownEcActual[2]; 

    int knownPhRaw[2]; 
    float knownPhActual[2]; 

    int knownTurbRaw[2]; 
    float knownTurbActual[2]; 
    
    int knownTdsRaw[2]; 
    float knownTdsActual[2]; 

    float ecSlope; 
    float ecInt; 

    float phSlope;
    float phInt; 
    
    float turbSlope;
    float turbInt; 

    float tdsSlope;
    float tdsInt; 
};

typedef struct AdcSensorConfig AdcSensorConfig_t;

extern float adc_to_msiemen_cm(int adc, AdcSensorConfig_t* cfg);
extern float adc_to_ph(int adc, AdcSensorConfig_t* cfg); 
extern float adc_to_turb(int adc, AdcSensorConfig_t* cfg); 
extern float adc_to_tds(int adc, AdcSensorConfig_t* cfg); 
extern int ec_get_adc();
extern int ph_get_adc(); 
extern int turb_get_adc(); 
extern int tds_get_adc(); 
extern esp_err_t analog_sensors_init(AdcSensorConfig_t* cfg);

extern int set_cal_if_suitable(char variable[], float realValue, AdcSensorConfig_t* cfg); 