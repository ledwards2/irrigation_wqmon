#include "nau7802.h"
const char* TAG = "NAU7802";



int nau7802_init(int sda_num, int scl_num, i2c_port_t port_num, struct nau7802_handle* handle) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER, 
        .sda_io_num = sda_num, 
        .sda_pullup_en = GPIO_PULLUP_DISABLE, 
        .scl_io_num = scl_num, 
        .scl_pullup_en = GPIO_PULLUP_DISABLE, 
        .master.clk_speed = NAU7802_I2C_FREQ, 
        .clk_flags = 0 
    };

    handle->conf = conf; 
    handle->portNum = port_num; 
    esp_err_t err = ESP_OK; 
    err |= i2c_param_config(port_num, &conf); 
    if (err) {
        ESP_LOGE(TAG, "i2c param config: %i", err); 
    }

    
    err |= i2c_driver_install(port_num, I2C_MODE_MASTER, 0, 0, 0);
    if (err) {
        ESP_LOGE(TAG, "i2c driver install: %i", err); 
    } else {
        ESP_LOGI(TAG, "I2c driver install success");
    }

    uint8_t read;
    readReg(handle, 0x0E, &read); 
    ESP_LOGE(TAG, "Shoudld be 0x80: %x", read); 
    nau7802_reset(handle);   
 
    if (nau7802_wait_power_up(handle) != ESP_OK) {
        ESP_LOGE(TAG, "Could not read power up"); 
        return ESP_FAIL; 
    }

    

    nau7802_configure(handle); 

    // nau7802_start_conversions(); 
    // nau7802_enable_exti();

    return err; 
}



int32_t nau7802_read_conversion(struct nau7802_handle* handle) {
    uint8_t buf[3]; 
    esp_err_t res; 
    
    readReg(handle, 0x12, &buf[0]);
    readReg(handle, 0x13, &buf[1]);
    readReg(handle, 0x14, &buf[2]);

    uint32_t raw = (uint32_t) buf[0] << 16 | (uint32_t) buf[1] << 8 | (uint32_t) buf[0]; 
    int32_t val = (int32_t) (raw << 8); 
    return val >> 8; 
}

void nau7802_configure(struct nau7802_handle* handle) {
    uint8_t ctrl = (1 << NAU7802_CTRL1_CRP) | (0b111 << NAU7802_CTRL1_VLDO) | (0b110 << NAU7802_CTRL1_GAIN); 

    writeReg(handle, NAU7802_CTRL1, ctrl); 

    uint8_t ctrl2 = (0 << NAU7802_CTRL2_CHS) | (000 << NAU7802_CTRL2_CRS | 0b000 << NAU7802_CTRL2_CRS); 
    writeReg(handle, NAU7802_CTRL2, ctrl2); 

    // setBit(handle, NAU7802_PGA, NAU7802_PGA_BYPASS_EN);
    
    setBit(handle, NAU7802_PU_CTRL, NAU7802_PU_CTRL_AVDDS); 
    setBit(handle, NAU7802_PU_CTRL, NAU7802_PU_CTRL_CS); 

    

}

// Tries 10 times, 100ms apart.
esp_err_t nau7802_wait_power_up(struct nau7802_handle* handle) {
    for (int attempt = 0; attempt < 10; attempt++) {
        setBit(handle, NAU7802_PU_CTRL, NAU7802_PU_CTRL_PUD); 
        setBit(handle, NAU7802_PU_CTRL, NAU7802_PU_CTRL_PUA); 


        ESP_LOGI(TAG, "Trying to power up: attempt %i", attempt); 
        vTaskDelay(100/portTICK_PERIOD_MS);

        if (getBit(handle, NAU7802_PU_CTRL, NAU7802_PU_CTRL_PUR) == true) {
            return ESP_OK; 
        }

    }
    return ESP_FAIL; 
}

void nau7802_reset(struct nau7802_handle* handle) {
    ESP_LOGI(TAG, "setting PU CTRL RR"); 
    setBit(handle, NAU7802_PU_CTRL, NAU7802_PU_CTRL_RR);
    
    vTaskDelay(RESET_DELAY);
    ESP_LOGI(TAG, "Clearing PU CTRL RR");
    clearBit(handle, NAU7802_PU_CTRL, NAU7802_PU_CTRL_RR); 
}

int32_t nau7802_get_most_recent();


void setBit(struct nau7802_handle* handle, uint8_t reg, uint8_t bit) {
    uint8_t original; 
    ESP_LOGI(TAG, "Setbit: %x, %x", reg, bit);
    esp_err_t res = readReg(handle, reg, &original);

    if (res) {
        ESP_LOGE(TAG, "setbit: %i", res); 
    }

    original |= 1 << bit; 

    res = writeReg(handle, reg, original); 

    if (res) {
        ESP_LOGE(TAG, "setbit: %i", res); 
    }
}

void clearBit(struct nau7802_handle* handle, uint8_t reg, uint8_t bit) {
    uint8_t original; 
    esp_err_t res = readReg(handle, reg, &original);

    if (res) {
        ESP_LOGE(TAG, "clearbit: %i", res); 
    }

    original &= ~(1 << bit); 

    writeReg(handle, reg, original); 

}

esp_err_t  writeReg(struct nau7802_handle* handle, uint8_t reg, uint8_t val) {
    uint8_t writeCommand[2];
    writeCommand[0] = reg; 
    writeCommand[1] = val;  
    /*esp_err_t status = i2c_master_write_read_device(
        handle->portNum, 
        NAU7802_DEV_ADDR,  
        val, 
        sizeof(int8_t), 
        &read, 
        sizeof(int8_t), 
        (TickType_t) (WAIT_TICKS)); 
    
    */ 
    esp_err_t status = i2c_master_write_to_device(
        handle->portNum, 
        NAU7802_DEV_ADDR, 
        &writeCommand[0], 
        2 * sizeof(int8_t),
        (TickType_t) (WAIT_TICKS));
    
    ESP_LOGI("writeReg", "writing %x: %x", reg, val);
    return status; 
}

esp_err_t readReg(struct nau7802_handle* handle, uint8_t addr, uint8_t* res) {

    
    esp_err_t status = i2c_master_write_read_device(
        handle->portNum, 
        NAU7802_DEV_ADDR, 
        &addr, 
        sizeof(int8_t), 
        res, 
        sizeof(res), 
        (TickType_t) (WAIT_TICKS)); 

    return status; 

}

bool getBit(struct nau7802_handle* handle, uint8_t addr, uint8_t bit) {
    uint8_t reg; 
    esp_err_t res = readReg(handle, addr, &reg);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "getBit failed: %i", res); 
        return false; 
    }

    if (reg & (1 << bit)) {
        return true; 
    }
    return false; 
}

float calculate_slope_weight(float x1, float y1, float x2, float y2) {
    return (y2 - y1) / (x2 - x1); 
}

float calculate_intercept_weight(float x1, float y1, float x2, float y2) {
    float slope = calculate_slope_weight(x1, y1, x2, y2); 

    return y1 - (slope * x1); 
}

float nau7802_get_weight(int32_t strain, NAU7802CalConfig_t* cfg) {
    return cfg->strainSlope * ((float) (strain)) + cfg->strainInt;
}

int set_cal_if_strain(char variable[], float realValue, NAU7802CalConfig_t* cfg, int32_t strain) {
    int success = CALIBRATE_STRAIN_FAIL; 
    // int32_t strain = nau7802_read_conversion(cfg->handle);

    if (!strcmp(variable, CAL_STRAIN_1)) {
        cfg->knownWeightActual[0] = realValue; 
        cfg->knownStrainRaw[0] = strain; 
        success = CALIBRATE_STRAIN_NO_CHANGE; 
    } else if (!strcmp(variable, CAL_STRAIN_2)) {
        cfg->knownWeightActual[1] = realValue; 
        cfg->knownStrainRaw[1] = strain;
        cfg->strainSlope = calculate_slope_weight(
            cfg->knownStrainRaw[0], 
            cfg->knownWeightActual[0], 
            cfg->knownStrainRaw[1],
            cfg->knownWeightActual[1]
        ); 

        cfg->strainInt = calculate_intercept_weight(
            cfg->knownStrainRaw[0], 
            cfg->knownWeightActual[0], 
            cfg->knownStrainRaw[1],
            cfg->knownWeightActual[1]
        );
        success = CALIBRATED_STRAIN; 
        //ESP_LOGI("knwon weigh 1: %f",cfg->knownWeightActual[0] );
        ESP_LOGI(TAG, "known weight 1: %f, 2: %f, strain 1: %li, strain 2: %li, slope: %f, int: %f", cfg->knownWeightActual[0], cfg->knownWeightActual[1], cfg->knownStrainRaw[0], cfg->knownStrainRaw[1],  cfg->strainSlope, cfg->strainInt);
    } 
    
    return success; 

} 
