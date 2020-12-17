/*
    bme280.h

    Created: 16/12/2020 4:27:09 PM
    Author: user
*/

#include <stdint.h>

#define BME_INIT_NO_ERR 1
#define BME_INIT_ERR 0

#define BME_READ_CAL_DONE 1
#define BME_IS_READ_CAL 0

#define BME_READ_TEMPERATURE_ERR -1

/*!
    @brief Register addresses
*/
typedef enum {
    BME280_REGISTER_DIG_T1 = 0x88,
    BME280_REGISTER_DIG_T2 = 0x8A,
    BME280_REGISTER_DIG_T3 = 0x8C,

    BME280_REGISTER_DIG_P1 = 0x8E,
    BME280_REGISTER_DIG_P2 = 0x90,
    BME280_REGISTER_DIG_P3 = 0x92,
    BME280_REGISTER_DIG_P4 = 0x94,
    BME280_REGISTER_DIG_P5 = 0x96,
    BME280_REGISTER_DIG_P6 = 0x98,
    BME280_REGISTER_DIG_P7 = 0x9A,
    BME280_REGISTER_DIG_P8 = 0x9C,
    BME280_REGISTER_DIG_P9 = 0x9E,

    BME280_REGISTER_DIG_H1 = 0xA1,
    BME280_REGISTER_DIG_H2 = 0xE1,
    BME280_REGISTER_DIG_H3 = 0xE3,
    BME280_REGISTER_DIG_H4 = 0xE4,
    BME280_REGISTER_DIG_H5 = 0xE5,
    BME280_REGISTER_DIG_H6 = 0xE7,

    BME280_REGISTER_CHIPID = 0xD0,
    BME280_REGISTER_VERSION = 0xD1,
    BME280_REGISTER_SOFTRESET = 0xE0,

    BME280_REGISTER_CAL26 = 0xE1,		// R calibration stored in 0xE1-0xF0

    BME280_REGISTER_STATUS = 0XF3,

    BME280_REGISTER_CONTROLHUMID = 0xF2,
    BME280_REGISTER_CONTROL = 0xF4,
    BME280_REGISTER_CONFIG = 0xF5,
    BME280_REGISTER_PRESSUREDATA = 0xF7,
    BME280_REGISTER_TEMPDATA = 0xFA,
    BME280_REGISTER_HUMIDDATA = 0xFD
} bme280_registers;

/**************************************************************************/
/*!
    @brief  sampling rates
*/
/**************************************************************************/
typedef enum {
    SAMPLING_NONE = 0b000,
    SAMPLING_X1 = 0b001,
    SAMPLING_X2 = 0b010,
    SAMPLING_X4 = 0b011,
    SAMPLING_X8 = 0b100,
    SAMPLING_X16 = 0b101
} sensor_sampling;

/**************************************************************************/
/*!
    @brief  power modes
*/
/**************************************************************************/
typedef enum {
    MODE_SLEEP = 0b00,
    MODE_FORCED = 0b01,
    MODE_NORMAL = 0b11
} sensor_mode;

/**************************************************************************/
/*!
    @brief  filter values
*/
/**************************************************************************/
typedef enum {
    FILTER_OFF = 0b000,
    FILTER_X2 = 0b001,
    FILTER_X4 = 0b010,
    FILTER_X8 = 0b011,
    FILTER_X16 = 0b100
} sensor_filter;

/**************************************************************************/
/*!
    @brief  standby duration in ms
*/
/**************************************************************************/
typedef enum  {
    STANDBY_MS_0_5 = 0b000,
    STANDBY_MS_10 = 0b110,
    STANDBY_MS_20 = 0b111,
    STANDBY_MS_62_5 = 0b001,
    STANDBY_MS_125 = 0b010,
    STANDBY_MS_250 = 0b011,
    STANDBY_MS_500 = 0b100,
    STANDBY_MS_1000 = 0b101
} standby_duration;

/**************************************************************************/
/*!
    @brief  calibration data struct type definition
*/
/**************************************************************************/
struct bme280_calib_data {
    uint16_t dig_T1; ///< temperature compensation value
    int16_t dig_T2;  ///< temperature compensation value
    int16_t dig_T3;  ///< temperature compensation value

    uint16_t dig_P1; ///< pressure compensation value
    int16_t dig_P2;  ///< pressure compensation value
    int16_t dig_P3;  ///< pressure compensation value
    int16_t dig_P4;  ///< pressure compensation value
    int16_t dig_P5;  ///< pressure compensation value
    int16_t dig_P6;  ///< pressure compensation value
    int16_t dig_P7;  ///< pressure compensation value
    int16_t dig_P8;  ///< pressure compensation value
    int16_t dig_P9;  ///< pressure compensation value

    uint8_t dig_H1; ///< humidity compensation value
    int16_t dig_H2; ///< humidity compensation value
    uint8_t dig_H3; ///< humidity compensation value
    int16_t dig_H4; ///< humidity compensation value
    int16_t dig_H5; ///< humidity compensation value
    int8_t dig_H6;  ///< humidity compensation value
} ;

/**************************************************************************/
/*!
    @brief  config struct type definition
*/
/**************************************************************************/
struct config {
    // inactive duration (standby time) in normal mode
    // 000 = 0.5 ms
    // 001 = 62.5 ms
    // 010 = 125 ms
    // 011 = 250 ms
    // 100 = 500 ms
    // 101 = 1000 ms
    // 110 = 10 ms
    // 111 = 20 ms
    unsigned int t_sb;

    // filter settings
    // 000 = filter off
    // 001 = 2x filter
    // 010 = 4x filter
    // 011 = 8x filter
    // 100 and above = 16x filter
    unsigned int filter;

    // unused - don't set
    unsigned int none;
    unsigned int spi3w_en;

    /// @return combined config register
    // NOTE: C structs cannot have functions, so this function is not used
    // Struct members will be read directly
    //unsigned int get() { return (t_sb << 5) | (filter << 2) | spi3w_en; }
} ;

/**************************************************************************/
/*!
    @brief  ctrl_meas register
*/
/**************************************************************************/
struct ctrl_meas {
    // temperature oversampling
    // 000 = skipped
    // 001 = x1
    // 010 = x2
    // 011 = x4
    // 100 = x8
    // 101 and above = x16
    unsigned int osrs_t;

    // pressure oversampling
    // 000 = skipped
    // 001 = x1
    // 010 = x2
    // 011 = x4
    // 100 = x8
    // 101 and above = x16
    unsigned int osrs_p;

    // device mode
    // 00       = sleep
    // 01 or 10 = forced
    // 11       = normal
    unsigned int mode;

    /// @return combined ctrl register
    // NOTE: C structs cannot have functions, so this function is not used
    //unsigned int get() { return (osrs_t << 5) | (osrs_p << 2) | mode; }
};

/**************************************************************************/
/*!
    @brief  ctrl_hum register
*/
/**************************************************************************/
struct ctrl_hum {
    /// unused - don't set
    unsigned int none;

    // pressure oversampling
    // 000 = skipped
    // 001 = x1
    // 010 = x2
    // 011 = x4
    // 100 = x8
    // 101 and above = x16
    unsigned int osrs_h;

    /// @return combined ctrl hum register
    // NOTE: C structs cannot have functions, so this function is not used
    //unsigned int get() { return (osrs_h); }
};



/*=========================================================================*/

int bme280_init(void);
uint8_t read8(uint8_t reg);
void write8 (uint8_t reg, uint8_t value);
uint16_t read16(uint8_t reg);
uint16_t read16_LE(uint8_t reg);
int16_t readS16(uint8_t reg) ;
int16_t readS16_LE(uint8_t reg);
uint32_t read24(uint8_t reg);
uint8_t spixfer(uint8_t x);
int is_reading_calibration(void);
void read_coefficients(void);
void set_sampling(sensor_mode mode, sensor_sampling temp_sampling,
                  sensor_sampling press_sampling, sensor_sampling hum_sampling,
                  sensor_filter filter, standby_duration duration);
float bme280_read_temperature(void);