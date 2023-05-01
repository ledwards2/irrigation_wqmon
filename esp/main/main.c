#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//#include "esp_system.h"
//#include "esp_spi_flash.h"

#include "wq_sensors.h"


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

