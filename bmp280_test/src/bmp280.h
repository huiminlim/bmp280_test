/*
    bmp280.h

    Created: 16/12/2020 4:27:09 PM
    Author: user
*/

#include <stdint.h>

#define BMP280_INIT_NO_ERR 1
#define BMP280_INIT_ERR 0

#define BMP280_READ_CAL_DONE 1
#define BMP280_IS_READ_CAL 0

#define BMP280_READ_TEMPERATURE_ERR -1

#define BMP280_CHIPID 0x58

/*!
    @brief Register addresses
*/
typedef enum {
    BMP280_REGISTER_DIG_T1 = 0x88,
    BMP280_REGISTER_DIG_T2 = 0x8A,
    BMP280_REGISTER_DIG_T3 = 0x8C,

    BMP280_REGISTER_DIG_P1 = 0x8E,
    BMP280_REGISTER_DIG_P2 = 0x90,
    BMP280_REGISTER_DIG_P3 = 0x92,
    BMP280_REGISTER_DIG_P4 = 0x94,
    BMP280_REGISTER_DIG_P5 = 0x96,
    BMP280_REGISTER_DIG_P6 = 0x98,
    BMP280_REGISTER_DIG_P7 = 0x9A,
    BMP280_REGISTER_DIG_P8 = 0x9C,
    BMP280_REGISTER_DIG_P9 = 0x9E,

    BMP280_REGISTER_CHIPID = 0xD0,
    BMP280_REGISTER_VERSION = 0xD1,
    BMP280_REGISTER_SOFTRESET = 0xE0,
    BMP280_REGISTER_CAL26 = 0xE1,				/**< R calibration = 0xE1-0xF0 */
    BMP280_REGISTER_STATUS = 0xF3,
    BMP280_REGISTER_CONTROL = 0xF4,
    BMP280_REGISTER_CONFIG = 0xF5,
    BMP280_REGISTER_PRESSUREDATA = 0xF7,
    BMP280_REGISTER_TEMPDATA = 0xFA
} bmp280_registers;

/**************************************************************************/
/*!
    @brief  sampling rates
*/
/**************************************************************************/
typedef enum {
    /** No over-sampling. */
    SAMPLING_NONE = 0x00,

    /** 1x over-sampling. */
    SAMPLING_X1 = 0x01,

    /** 2x over-sampling. */
    SAMPLING_X2 = 0x02,

    /** 4x over-sampling. */
    SAMPLING_X4 = 0x03,

    /** 8x over-sampling. */
    SAMPLING_X8 = 0x04,

    /** 16x over-sampling. */
    SAMPLING_X16 = 0x05
} sensor_sampling;

/**************************************************************************/
/*!
    @brief  power modes
*/
/**************************************************************************/
typedef enum {
    /** Sleep mode. */
    MODE_SLEEP = 0x00,

    /** Forced mode. */
    MODE_FORCED = 0x01,

    /** Normal mode. */
    MODE_NORMAL = 0x03,

    /** Software reset. */
    MODE_SOFT_RESET_CODE = 0xB6
} sensor_mode;

/**************************************************************************/
/*!
    @brief  filter values
*/
/**************************************************************************/
typedef enum {
    /** No filtering. */
    FILTER_OFF = 0x00,

    /** 2x filtering. */
    FILTER_X2 = 0x01,

    /** 4x filtering. */
    FILTER_X4 = 0x02,

    /** 8x filtering. */
    FILTER_X8 = 0x03,

    /** 16x filtering. */
    FILTER_X16 = 0x04
} sensor_filter;

/**************************************************************************/
/*!
    @brief  standby duration in ms
*/
/**************************************************************************/
typedef enum  {
    /** 1 ms standby. */
    STANDBY_MS_1 = 0x00,

    /** 62.5 ms standby. */
    STANDBY_MS_63 = 0x01,

    /** 125 ms standby. */
    STANDBY_MS_125 = 0x02,

    /** 250 ms standby. */
    STANDBY_MS_250 = 0x03,

    /** 500 ms standby. */
    STANDBY_MS_500 = 0x04,

    /** 1000 ms standby. */
    STANDBY_MS_1000 = 0x05,

    /** 2000 ms standby. */
    STANDBY_MS_2000 = 0x06,

    /** 4000 ms standby. */
    STANDBY_MS_4000 = 0x07
} standby_duration;

/**************************************************************************/
/*!
    @brief  calibration data struct type definition
*/
/**************************************************************************/
struct bmp280_calib_data {
    uint16_t dig_T1;			/**< dig_T1 cal register. */
    int16_t dig_T2;				/**<  dig_T2 cal register. */
    int16_t dig_T3;				/**< dig_T3 cal register. */

    uint16_t dig_P1;			/**< dig_P1 cal register. */
    int16_t dig_P2;				/**< dig_P2 cal register. */
    int16_t dig_P3;				/**< dig_P3 cal register. */
    int16_t dig_P4;				/**< dig_P4 cal register. */
    int16_t dig_P5;				/**< dig_P5 cal register. */
    int16_t dig_P6;				/**< dig_P6 cal register. */
    int16_t dig_P7;				/**< dig_P7 cal register. */
    int16_t dig_P8;				/**< dig_P8 cal register. */
    int16_t dig_P9;				/**< dig_P9 cal register. */
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


/*=========================================================================*/

int bmp280_init(void);
uint8_t read8(uint8_t reg);
void write8 (uint8_t reg, uint8_t value);
uint16_t read16(uint8_t reg);
uint16_t read16_LE(uint8_t reg);
int16_t readS16(uint8_t reg) ;
int16_t readS16_LE(uint8_t reg);
uint32_t read24(uint8_t reg);
uint8_t spixfer(uint8_t x);
void read_coefficients(void);
void set_sampling(sensor_mode mode, sensor_sampling temp_sampling,
                  sensor_sampling press_sampling, sensor_filter filter, standby_duration duration);
int32_t bmp280_read_temperature(void);
int64_t bmp280_read_pressure(void);