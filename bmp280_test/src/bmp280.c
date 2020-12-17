/*
    bme280.c

    Created: 16/12/2020 4:27:01 PM
    Author: user
*/

#include <bme280.h>
#include <spi.h>
#include <uart.h>
#include <delay.h>

// Structs defined
struct bme280_calib_data bme280_calib_data_read;

struct config config_reg = {
    .t_sb = 3,					///< inactive duration (standby time) in normal mode
    .filter = 3,				///< filter settings
    .none = 1,					///< unused - don't set
    .spi3w_en = 1				///< unused - don't set
};								//!< config register object

struct ctrl_meas meas_reg = {
    .osrs_t = 3,				///< temperature oversampling
    .osrs_p = 3,				///< pressure oversampling
    .mode = 2					///< device mode
};								//!< measurement register object

struct ctrl_hum hum_reg = {
    .none = 5,
    .osrs_h = 3					///< pressure oversampling
};								//!< hum register object

/*
    This function verifies if sensor is initialized
    Assumes that SPI has been initialized already
*/
int bme280_init(void) {
    // Check if read successfully from sensor ID
    //uint8_t sensor_id = read8(BME280_REGISTER_CHIPID);

    //printf("Sensor ID: 0x%x\r\n", sensor_id);

    //if (sensor_id != 0x60) {
    //    return BME_INIT_ERR;
    //}


    // Reset device with soft-reset
    // Make sure that IIR is off, etc
    write8(BME280_REGISTER_SOFTRESET, 0xB6);

    // Wait for device to wake up
    delay_ms(10);

    // If the chip is still reading calibration, delay
    while (is_reading_calibration() != BME_READ_CAL_DONE) {
        delay_ms(100);
    }

    // read trimming parameters
    // See Datasheet 4.2.2
    read_coefficients();

    // Set default sampling
    set_sampling(MODE_NORMAL, SAMPLING_X16,
                 SAMPLING_X16, SAMPLING_X16, FILTER_OFF,
                 STANDBY_MS_0_5);

    delay_ms(200);

    return BME_INIT_NO_ERR;
}

/*!
     @brief  Returns the temperature from the sensor
     @returns the temperature read from the device
*/
float bme280_read_temperature(void) {
    int32_t var1, var2;
    int32_t adc_T = read24(BME280_REGISTER_TEMPDATA);

    // value in case temp measurement was disabled
    if (adc_T == 0x800000) {
        return BME_READ_TEMPERATURE_ERR;
    }

    // Problem is here
    adc_T = (adc_T >> 4);

    printf("adc: %ld\r\n", adc_T);

    // Calibrate the temperature sensor data
    var1 = ((((adc_T >> 3) - ((int32_t) bme280_calib_data_read.dig_T1 << 1))) * ((
                int32_t) bme280_calib_data_read.dig_T2)) >> 11;

    var2 = (((((adc_T >> 4) - ((int32_t) bme280_calib_data_read.dig_T1)) * (( adc_T >> 4) - ((
                  int32_t) bme280_calib_data_read.dig_T1))) >> 12) * ((int32_t) bme280_calib_data_read.dig_T3)) >> 14;

    //!< add to compensate temp readings and in turn
    //!< to pressure and humidity readings
    int32_t t_fine_adjust = 0;

    int32_t t_fine = var1 + var2 + t_fine_adjust;
    float T = (t_fine * 5 + 128) >> 8;
    return T / 100;
}

/*
    This function checks if device is still busy reading calibration
*/
int is_reading_calibration(void) {
    uint8_t read_status = read8(BME280_REGISTER_STATUS);

    if ((read_status & (1 << 0)) == 0) {
        return BME_IS_READ_CAL;
    }

    return BME_READ_CAL_DONE;
}

/*!
     @brief  Reads the factory-set coefficients
*/
void read_coefficients(void) {
    bme280_calib_data_read.dig_T1 = read16_LE(BME280_REGISTER_DIG_T1);
    bme280_calib_data_read.dig_T2 = readS16_LE(BME280_REGISTER_DIG_T2);
    bme280_calib_data_read.dig_T3 = readS16_LE(BME280_REGISTER_DIG_T3);
    bme280_calib_data_read.dig_P1 = read16_LE(BME280_REGISTER_DIG_P1);
    bme280_calib_data_read.dig_P2 = readS16_LE(BME280_REGISTER_DIG_P2);
    bme280_calib_data_read.dig_P3 = readS16_LE(BME280_REGISTER_DIG_P3);
    bme280_calib_data_read.dig_P4 = readS16_LE(BME280_REGISTER_DIG_P4);
    bme280_calib_data_read.dig_P5 = readS16_LE(BME280_REGISTER_DIG_P5);
    bme280_calib_data_read.dig_P6 = readS16_LE(BME280_REGISTER_DIG_P6);
    bme280_calib_data_read.dig_P7 = readS16_LE(BME280_REGISTER_DIG_P7);
    bme280_calib_data_read.dig_P8 = readS16_LE(BME280_REGISTER_DIG_P8);
    bme280_calib_data_read.dig_P9 = readS16_LE(BME280_REGISTER_DIG_P9);
    bme280_calib_data_read.dig_H1 = read8(BME280_REGISTER_DIG_H1);
    bme280_calib_data_read.dig_H2 = readS16_LE(BME280_REGISTER_DIG_H2);
    bme280_calib_data_read.dig_H3 = read8(BME280_REGISTER_DIG_H3);
    bme280_calib_data_read.dig_H4 = ((int8_t)read8(BME280_REGISTER_DIG_H4) << 4) | (read8(
                                        BME280_REGISTER_DIG_H4 + 1) & 0xF);
    bme280_calib_data_read.dig_H5 = ((int8_t)read8(BME280_REGISTER_DIG_H5 + 1) << 4) | (read8(
                                        BME280_REGISTER_DIG_H5) >> 4);
    bme280_calib_data_read.dig_H6 = (int8_t)read8(
                                        BME280_REGISTER_DIG_H6);
}

/*!
     @brief  setup sensor with given parameters / settings

     This is simply a overload to the normal begin()-function, so SPI users
     don't get confused about the library requiring an address.
     @param mode the power mode to use for the sensor
     @param tempSampling the temp sampling rate to use
     @param pressSampling the pressure sampling rate to use
     @param humSampling the humidity sampling rate to use
     @param filter the filter mode to use
     @param duration the standby duration to use
*/
void set_sampling(sensor_mode mode,
                  sensor_sampling temp_sampling,
                  sensor_sampling press_sampling,
                  sensor_sampling hum_sampling,
                  sensor_filter filter, standby_duration duration) {

    meas_reg.mode = mode;
    meas_reg.osrs_t = temp_sampling;
    meas_reg.osrs_p = press_sampling;

    hum_reg.osrs_h = hum_sampling;

    config_reg.filter = filter;
    config_reg.t_sb = duration;

    // making sure sensor is in sleep mode before setting configuration
    // as it otherwise may be ignored
    write8(BME280_REGISTER_CONTROL, MODE_SLEEP);

    // you must make sure to also set REGISTER_CONTROL after setting the
    // CONTROLHUMID register, otherwise the values won't be applied (see
    // DS 5.4.3)
    //write8(BME280_REGISTER_CONTROLHUMID, _humReg.get());
    unsigned int hum_reg_val = hum_reg.osrs_h;
    write8(BME280_REGISTER_CONTROLHUMID, hum_reg_val);

    //write8(BME280_REGISTER_CONFIG, _configReg.get());
    unsigned int config_reg_val = ((config_reg.t_sb << 5) | (config_reg.filter << 2) | config_reg.spi3w_en);
    write8(BME280_REGISTER_CONFIG, config_reg_val);

    //write8(BME280_REGISTER_CONTROL, _measReg.get());
    unsigned int meas_reg_val = ((meas_reg.osrs_t << 5) | (meas_reg.osrs_p << 2) | meas_reg.mode);
    write8(BME280_REGISTER_CONTROL, meas_reg_val);
}

/*
    This function reads 8 bits from sensor
    With a given reg
*/
uint8_t read8(uint8_t reg) {
    spi_begin_txn(500000, MSBFIRST, SPI_MODE0);
    // SS set to low - select slave
    ioport_set_pin_low(SPI_HARDWARE_SS);

    // read, bit 7 set to 1
    spixfer(reg | 0x80);
    uint8_t value = spixfer(0);

    // SS set to high - de-select slave
    ioport_set_pin_high(SPI_HARDWARE_SS);
    //spi_end_txn();

    return value;
}

/*
    This function writes 8 bits to sensor
    With a given reg
*/
void write8 (uint8_t reg, uint8_t value) {
    spi_begin_txn(500000, MSBFIRST, SPI_MODE0);

    // SS set to low - select slave
    ioport_set_pin_low(SPI_HARDWARE_SS);

    // read, bit 7 set to 0
    spixfer(reg & ~0x80);
    spixfer(value);

    // SS set to high - de-select slave
    ioport_set_pin_high(SPI_HARDWARE_SS);
    //spi_end_txn();
}

/*!
     @brief  Reads a 16 bit value over I2C or SPI
     @param reg the register address to read from
     @returns the 16 bit data value read from the device
*/
uint16_t read16(uint8_t reg) {
    uint16_t value;
    spi_begin_txn(500000, MSBFIRST, SPI_MODE0);

    // SS set to low - select slave
    ioport_set_pin_low(SPI_HARDWARE_SS);
    // read, bit 7 set to 1
    spixfer(reg | 0x80);
    value = (spixfer(0) << 8) | spixfer(0);

    // SS set to high - de-select slave
    ioport_set_pin_high(SPI_HARDWARE_SS);
    spi_end_txn();

    return value;
}

/*!
     @brief  Reads a signed 16 bit little endian value over I2C or SPI
     @param reg the register address to read from
     @returns the 16 bit data value read from the device
*/
uint16_t read16_LE(uint8_t reg) {
    uint16_t temp = read16(reg);
    return (temp >> 8) | (temp << 8);
}

/*!
     @brief  Reads a signed 16 bit value over I2C or SPI
     @param reg the register address to read from
     @returns the 16 bit data value read from the device
*/
int16_t readS16(uint8_t reg) {
    return (int16_t)read16(reg);
}


/*!
     @brief  Reads a signed little endian 16 bit value over I2C or SPI
     @param reg the register address to read from
     @returns the 16 bit data value read from the device
*/
int16_t readS16_LE(uint8_t reg) {
    return (int16_t)read16_LE(reg);
}

/*!
     @brief  Reads a 24 bit value
     @param reg the register address to read from
     @returns the 24 bit data value read from the device
*/
uint32_t read24(uint8_t reg) {
    uint32_t value;
    spi_begin_txn(500000, MSBFIRST, SPI_MODE0);

    // SS set to low - select slave
    ioport_set_pin_low(SPI_HARDWARE_SS);

    // read, bit 7 high
    spixfer(reg | 0x80);
    value = spixfer(0);
    value <<= 8;
    value |= spixfer(0);
    value <<= 8;
    value |= spixfer(0);

    // SS set to high - de-select slave
    ioport_set_pin_high(SPI_HARDWARE_SS);
    spi_end_txn();

    return value;
}

/*
    This function transfers 8 bits via SPI
*/
uint8_t spixfer(uint8_t x) {
    // Hardware SPI transfer used
    // Write to MOSI pin and receive on MISO pin
    SPDR = x;

    // NOP to introduce delay to prevent wait
    // Loop form iterating when running at the maximum speed
    // This gives about 10% more speed,
    // even if it seems counter-intuitive at lower speeds it is unnoticed.
    asm volatile("nop");

    while (!(SPSR & (1 << SPIF))) ; // wait

    return SPDR;
}




