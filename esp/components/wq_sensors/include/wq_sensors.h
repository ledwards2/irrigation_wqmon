
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include "driver/adc.h"

#define EC_ADC_WIDTH ADC_WIDTH_BIT_12
#define EC_ADC_CHANNEL ADC1_CHANNEL_0
#define EC_ADC_ATTEN_DB ADC_ATTEN_DB_11


extern float adc_to_msiemen_cm(int adc);
extern int ec_get_adc();
extern esp_err_t ec_sensor_init();


