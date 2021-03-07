/*
    bmp280.h

    Created: 16/12/2020 4:27:09 PM
    Author: user
*/

#include <stdint.h>
#include <spi.h>
#include <uart.h>
#include <delay.h>

#define BMP280_INIT_NO_ERR 1
#define BMP280_INIT_ERR 0

#define BMP280_READ_CAL_DONE 1
#define BMP280_IS_READ_CAL 0

#define BMP280_READ_TEMPERATURE_ERR -1

#define BMP280_CHIPID 0x58

/*!
    @brief Register addresses
*/
#define BMP280_REGISTER_DIG_T1 0x88
#define BMP280_REGISTER_DIG_T2 0x8A
#define BMP280_REGISTER_DIG_T3 0x8C
#define BMP280_REGISTER_DIG_P1 0x8E
#define BMP280_REGISTER_DIG_P2 0x90
#define BMP280_REGISTER_DIG_P3 0x92
#define BMP280_REGISTER_DIG_P4 0x94
#define BMP280_REGISTER_DIG_P5 0x96
#define BMP280_REGISTER_DIG_P6 0x98
#define BMP280_REGISTER_DIG_P7 0x9A
#define BMP280_REGISTER_DIG_P8 0x9C
#define BMP280_REGISTER_DIG_P9 0x9E

#define BMP280_REGISTER_CHIPID			0xD0
#define BMP280_REGISTER_VERSION			0xD1
#define BMP280_REGISTER_SOFTRESET		0xE0
#define BMP280_REGISTER_CAL26			0xE1				/**< R calibration = 0xE1-0xF0 */
#define BMP280_REGISTER_STATUS			0xF3
#define BMP280_REGISTER_CONTROL			0xF4
#define BMP280_REGISTER_CONFIG			0xF5
#define BMP280_REGISTER_PRESSUREDATA	0xF7
#define BMP280_REGISTER_TEMPDATA		0xFA


/**************************************************************************/
/*!
    @brief  sampling rates
*/
/**************************************************************************/
/** No over-sampling. */
#define SAMPLING_NONE 0x00

/** 1x over-sampling. */
#define SAMPLING_X1 0x01

/** 2x over-sampling. */
#define SAMPLING_X2 0x02

/** 4x over-sampling. */
#define SAMPLING_X4 0x03

/** 8x over-sampling. */
#define SAMPLING_X8 0x04

/** 16x over-sampling. */
#define SAMPLING_X16 0x05

/**************************************************************************/
/*!
    @brief  power modes
*/
/**************************************************************************/

/** Sleep mode. */
#define MODE_SLEEP 0x00

/** Forced mode. */
#define MODE_FORCED 0x01

/** Normal mode. */
#define MODE_NORMAL 0x03

/** Software reset. */
#define MODE_SOFT_RESET_CODE 0xB6

/**************************************************************************/
/*!
    @brief  filter values
*/
/**************************************************************************/

/** No filtering. */
#define FILTER_OFF 0x00

/** 2x filtering. */
#define FILTER_X2 0x01

/** 4x filtering. */
#define FILTER_X4 0x02

/** 8x filtering. */
#define FILTER_X8 0x03

/** 16x filtering. */
#define FILTER_X16 0x04

/**************************************************************************/
/*!
    @brief  standby duration in ms
*/
/**************************************************************************/

/** 1 ms standby. */
#define STANDBY_MS_1 0x00

/** 62.5 ms standby. */
#define STANDBY_MS_63 0x01

/** 125 ms standby. */
#define STANDBY_MS_125 0x02

/** 250 ms standby. */
#define STANDBY_MS_250 0x03

/** 500 ms standby. */
#define STANDBY_MS_500 0x04

/** 1000 ms standby. */
#define STANDBY_MS_1000 0x05

/** 2000 ms standby. */
#define STANDBY_MS_2000 0x06

/** 4000 ms standby. */
#define STANDBY_MS_4000 0x07

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
void print_coefficients(void);
int32_t bmp280_read_temperature(void);
int64_t bmp280_read_pressure(void);
void bmp280_set_ctrl(uint8_t osrs_t, uint8_t osrs_p, uint8_t mode);
void bmp280_set_config(uint8_t t_sb, uint8_t filter, uint8_t spi3w_en);