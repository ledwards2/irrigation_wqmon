#include "wq_sensors.h"
esp_err_t analog_sensors_init(AdcSensorConfig_t* cfg) {

    cfg->ecChannel = EC_ADC_CHANNEL; 
    cfg->ecAtten = EC_ADC_ATTEN_DB; 
    cfg->phChannel = PH_ADC_CHANNEL; 
    cfg->phAtten = PH_ADC_ATTEN_DB; 
    cfg->turbChannel = TURB_ADC_CHANNEL; 
    cfg->turbAtten = TURB_ADC_ATTEN_DB; 
    cfg->tdsChannel = TDS_ADC_CHANNEL; 
    cfg->tdsAtten = TDS_ADC_ATTEN_DB; 

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

float adc_to_msiemen_cm(int adc, AdcSensorConfig_t* cfg) {
    //float reading = ((float) adc) * 0.005673439 + 0.114762141;
    return ((float) (adc)) * cfg->ecSlope + cfg->ecInt;
    // return reading; 
}

float adc_to_ph(int adc, AdcSensorConfig_t* cfg) {
    return ((float) (adc)) * cfg->phSlope + cfg->phInt;
}

float adc_to_turb(int adc, AdcSensorConfig_t* cfg) {
    return ((float) (adc)) * cfg->turbSlope + cfg->turbInt;
}

float adc_to_tds(int adc, AdcSensorConfig_t* cfg) {
    return ((float) (adc)) *  cfg->tdsSlope + cfg->tdsInt;
}

int ec_get_adc() {
    return adc_get_avg(EC_ADC_CHANNEL, 10);
}

int ph_get_adc() {
    return adc_get_avg(PH_ADC_CHANNEL, 10);
}

int turb_get_adc() {
    return adc_get_avg(TURB_ADC_CHANNEL, 10); 
}

int tds_get_adc() {
    return adc_get_avg(TDS_ADC_CHANNEL, 10);
}

int adc_get_avg(adc1_channel_t chan, int numSamples) {
    long int tot = 0; 
    for (int i = 1; i < numSamples; i++) {
        tot = tot + adc1_get_raw(chan); 
    }

    return (int) (tot / numSamples); 
}


// Read from filename, populate value, return error code. 
int get_calibration_value(char* filename, float* value) {
    return -1; 
}


float calculate_slope(float x1, float y1, float x2, float y2) {
    return (y2 - y1) / (x2 - x1); 
}

float calculate_intercept(float x1, float y1, float x2, float y2) {
    float slope = calculate_slope(x1, y1, x2, y2); 

    return y1 - (slope * x1); 
}

int set_cal_if_suitable(char variable[], float realValue, AdcSensorConfig_t* cfg) {
    int ret = CALIBRATE_FAIL; 
    int adcRead; 
    if (!strcmp(variable, EC_CAL_1)) {
        adcRead = ec_get_adc(); 
        cfg->knownEcActual[0] = realValue; 
        cfg->knownEcRaw[0] = adcRead; 
        ret = CALIBRATE_NO_CHANGE; 
    } else if (!strcmp(variable, EC_CAL_2)) {
        adcRead = ec_get_adc(); 
        cfg->knownEcActual[1] = realValue; 
        cfg->knownEcRaw[1] = adcRead;

        cfg->ecSlope = calculate_slope(
            cfg->knownEcRaw[0], 
            cfg->knownEcActual[0], 
            cfg->knownEcRaw[1],
            cfg->knownEcActual[1]
        ); 

        cfg->ecInt = calculate_intercept( 
            cfg->knownEcRaw[0], 
            cfg->knownEcActual[0], 
            cfg->knownEcRaw[1],
            cfg->knownEcActual[1]
        ); 

        ret = CALIBRATED_EC; 

    } else if (!strcmp(variable, PH_CAL_1)) {
        adcRead = ph_get_adc(); 
        cfg->knownPhActual[0] = realValue; 
        cfg->knownPhRaw[0] = adcRead; 
        ret = CALIBRATE_NO_CHANGE; 

    } else if (!strcmp(variable, PH_CAL_2)) {
        adcRead = ph_get_adc(); 
        cfg->knownPhActual[1] = realValue; 
        cfg->knownPhRaw[1] = adcRead; 
        cfg->phSlope = calculate_slope(
            cfg->knownPhRaw[0], 
            cfg->knownPhActual[0], 
            cfg->knownPhRaw[1],
            cfg->knownPhActual[1]
        ); 

        cfg->phInt = calculate_intercept( 
            cfg->knownPhRaw[0], 
            cfg->knownPhActual[0], 
            cfg->knownPhRaw[1],
            cfg->knownPhActual[1]
        ); 

        ret = CALIBRATED_PH; 
    } else if (!strcmp(variable, TURB_CAL_1)) {
        adcRead = turb_get_adc(); 
        cfg->knownTurbActual[0] = realValue; 
        cfg->knownTurbRaw[0] = adcRead; 
        ret = CALIBRATE_NO_CHANGE; 

    } else if (!strcmp(variable, TURB_CAL_2)) {
        adcRead = turb_get_adc(); 
        cfg->knownTurbActual[1] = realValue; 
        cfg->knownTurbRaw[1] = adcRead; 

         cfg->tdsSlope = calculate_slope(
            cfg->knownTdsRaw[0], 
            cfg->knownTdsActual[0], 
            cfg->knownTdsRaw[1],
            cfg->knownTdsActual[1]
        ); 

        cfg->tdsInt = calculate_intercept( 
            cfg->knownTdsRaw[0], 
            cfg->knownTdsActual[0], 
            cfg->knownTdsRaw[1],
            cfg->knownTdsActual[1]
        ); 
        ret = CALIBRATED_TURB; 
    } else if (!strcmp(variable, TDS_CAL_1)) {
        adcRead = tds_get_adc(); 
        cfg->knownTdsActual[0] = realValue; 
        cfg->knownTdsRaw[0] = adcRead; 
        ret = CALIBRATE_NO_CHANGE; 

    } else if (!strcmp(variable, TDS_CAL_2)) {
        adcRead = tds_get_adc(); 
        cfg->knownTdsActual[1] = realValue; 
        cfg->knownTdsRaw[1] = adcRead; 

        cfg->tdsSlope = calculate_slope(
            cfg->knownTdsRaw[0], 
            cfg->knownTdsActual[0], 
            cfg->knownTdsRaw[1],
            cfg->knownTdsActual[1]
        ); 

        cfg->tdsInt = calculate_intercept( 
            cfg->knownTdsRaw[0], 
            cfg->knownTdsActual[0], 
            cfg->knownTdsRaw[1],
            cfg->knownTdsActual[1]
        ); 
        ret = CALIBRATED_TDS; 
    } else {
        // Not suitable - flag that this isn't an ADC sensor.
        ret = CALIBRATE_NO_CHANGE; 
    } 
    return ret; 
}