#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//#include "esp_system.h"
//#include "esp_spi_flash.h"

#include "wq_sensors.h"
#include "mqtt_comms.h"

#include <string.h>

void app_main(void)
{
    printf("Hello world!\n");

    analog_sensors_init(); 
    wifi_init();
    mqtt_init();
    int reading;
    float conductivity; 
    struct tago_msg msg;
    memcpy(&msg.unit, "mS", 3);
    memcpy(&msg.variable, "EC", 3);
 
    while (1)  {
        reading = adc1_get_raw(ADC1_CHANNEL_0); 
        conductivity = adc_to_msiemen_cm(reading); 
        printf("Reading: %fmS/cm\n", conductivity); 
        msg.value = conductivity;
        tago_send(msg);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

    }

}

