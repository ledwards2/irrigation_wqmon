#include "driver/i2c.h"
#include <esp_log.h>
#include <stdbool.h>
#include <string.h>
#define NAU7802_DEV_ADDR 0x2A // might be 0x15 depending on endianness. 
#define WAIT_TICKS 10
#define RESET_DELAY 1

#define NAU7802_I2C_FREQ 200000

#define CAL_STRAIN_2 "cali_weight_2"
#define CAL_STRAIN_1 "cali_weight_1"
#define CALIBRATE_STRAIN_FAIL 1 
#define CALIBRATE_STRAIN_NO_CHANGE 2  
#define CALIBRATED_STRAIN 3 
struct nau7802_handle {
    i2c_config_t conf;
    i2c_port_t portNum;
};
struct NAU7802CalConfig {
	int32_t knownStrainRaw[2]; 
	float knownWeightActual[2]; 

	float strainSlope; 
	float strainInt; 
	struct nau7802_handle* handle; 
}; 

typedef struct NAU7802CalConfig NAU7802CalConfig_t; 

int nau7802_init(int sda_num, int scl_num, i2c_port_t port_num, struct nau7802_handle* handle);
void nau7802_reset(struct nau7802_handle* handle);
void nau7802_configure(struct nau7802_handle* handle);

esp_err_t nau7802_wait_power_up(struct nau7802_handle* handle);
void setBit(struct nau7802_handle* handle, uint8_t reg, uint8_t bit);
void clearBit(struct nau7802_handle* handle, uint8_t reg, uint8_t bit);
esp_err_t  writeReg(struct nau7802_handle* handle, uint8_t reg, uint8_t val);
esp_err_t readReg(struct nau7802_handle* handle, uint8_t addr, uint8_t* res);
bool getBit(struct nau7802_handle* handle, uint8_t addr, uint8_t bit); 
extern int32_t nau7802_read_conversion(struct nau7802_handle* handle); 
float calculate_slope_weight(float x1, float y1, float x2, float y2); 
float calculate_intercept_weight(float x1, float y1, float x2, float y2);
extern int set_cal_if_strain(char variable[], float realValue, NAU7802CalConfig_t* cfg);

float nau7802_get_weight(int32_t strain, NAU7802CalConfig_t* cfg);

/**
 * Enum used to define the scale registers
 */
typedef enum
{
	NAU7802_PU_CTRL = 0x00,
	NAU7802_CTRL1,
	NAU7802_CTRL2,
	NAU7802_OCAL1_B2,
	NAU7802_OCAL1_B1,
	NAU7802_OCAL1_B0,
	NAU7802_GCAL1_B3,
	NAU7802_GCAL1_B2,
	NAU7802_GCAL1_B1,
	NAU7802_GCAL1_B0,
	NAU7802_OCAL2_B2,
	NAU7802_OCAL2_B1,
	NAU7802_OCAL2_B0,
	NAU7802_GCAL2_B3,
	NAU7802_GCAL2_B2,
	NAU7802_GCAL2_B1,
	NAU7802_GCAL2_B0,
	NAU7802_I2C_CONTROL,
	NAU7802_ADCO_B2,
	NAU7802_ADCO_B1,
	NAU7802_ADCO_B0,
	NAU7802_ADC = 0x15, //Shared ADC and OTP 32:24
	NAU7802_OTP_B1,     //OTP 23:16 or 7:0?
	NAU7802_OTP_B0,     //OTP 15:8
	NAU7802_PGA = 0x1B,
	NAU7802_PGA_PWR = 0x1C,
	NAU7802_DEVICE_REV = 0x1F,
} Scale_Registers;

/**
 * Enum used to define the bits in the PU_CTRL register
 */
typedef enum
{
	NAU7802_PU_CTRL_RR = 0,
	NAU7802_PU_CTRL_PUD = 1,
	NAU7802_PU_CTRL_PUA = 2,
	NAU7802_PU_CTRL_PUR = 3,
	NAU7802_PU_CTRL_CS = 4,
	NAU7802_PU_CTRL_CR = 5,
	NAU7802_PU_CTRL_OSCS = 6,
	NAU7802_PU_CTRL_AVDDS = 7,
} PU_CTRL_Bits;

/**
 * Enum used to define the bits in the CTRL1 register
 */
typedef enum
{
	NAU7802_CTRL1_GAIN = 2,
	NAU7802_CTRL1_VLDO = 5,
	NAU7802_CTRL1_DRDY_SEL = 6,
	NAU7802_CTRL1_CRP = 7,
} CTRL1_Bits;

/**
 * Enum used to define the bits in the CTRL2 register
 */
typedef enum
{
	NAU7802_CTRL2_CALMOD = 0,
	NAU7802_CTRL2_CALS = 2,
	NAU7802_CTRL2_CAL_ERROR = 3,
	NAU7802_CTRL2_CRS = 4,
	NAU7802_CTRL2_CHS = 7,
} CTRL2_Bits;

/**
 * Enum used to define the bits in the PGA register
 */
typedef enum
{
	NAU7802_PGA_CHP_DIS = 0,
	NAU7802_PGA_INV = 3,
	NAU7802_PGA_BYPASS_EN,
	NAU7802_PGA_OUT_EN,
	NAU7802_PGA_LDOMODE,
	NAU7802_PGA_RD_OTP_SEL,
} PGA_Bits;

/**
 * Enum used to define the bits in the PGA_PWR register
 */
typedef enum
{
	NAU7802_PGA_PWR_PGA_CURR = 0,
	NAU7802_PGA_PWR_ADC_CURR = 2,
	NAU7802_PGA_PWR_MSTR_BIAS_CURR = 4,
	NAU7802_PGA_PWR_PGA_CAP_EN = 7,
} PGA_PWR_Bits;

/**
 * Enum used to define the bits setting of different LDO values
 */
typedef enum
{
	NAU7802_LDO_2V4 = 0b111,
	NAU7802_LDO_2V7 = 0b110,
	NAU7802_LDO_3V0 = 0b101,
	NAU7802_LDO_3V3 = 0b100,
	NAU7802_LDO_3V6 = 0b011,
	NAU7802_LDO_3V9 = 0b010,
	NAU7802_LDO_4V2 = 0b001,
	NAU7802_LDO_4V5 = 0b000,
} NAU7802_LDO_Values;

/**
 * Enum used to define the bits setting of different gain values
 */
typedef enum
{
	NAU7802_GAIN_128 = 0b111,
	NAU7802_GAIN_64 = 0b110,
	NAU7802_GAIN_32 = 0b101,
	NAU7802_GAIN_16 = 0b100,
	NAU7802_GAIN_8 = 0b011,
	NAU7802_GAIN_4 = 0b010,
	NAU7802_GAIN_2 = 0b001,
	NAU7802_GAIN_1 = 0b000,
} NAU7802_Gain_Values;

/**
 * Enum used to define the bits of different SPS settings
 */
typedef enum
{
	NAU7802_SPS_320 = 0b111,
	NAU7802_SPS_80 = 0b011,
	NAU7802_SPS_40 = 0b010,
	NAU7802_SPS_20 = 0b001,
	NAU7802_SPS_10 = 0b000,
} NAU7802_SPS_Values;

/**
 * Enum used to define the channels
 */
typedef enum
{
	NAU7802_CHANNEL_1 = 0, NAU7802_CHANNEL_2 = 1,
} NAU7802_Channels;

/**
 * Enum used to define calibration status
 */
typedef enum
{
	NAU7802_CAL_SUCCESS = 0,
	NAU7802_CAL_IN_PROGRESS = 1,
	NAU7802_CAL_FAILURE = 2,
} NAU7802_Cal_Status;

extern int nau7802_init(
	int sda_num, 
	int scl_num, 
	i2c_port_t port_num, 
	struct nau7802_handle* handle); 



