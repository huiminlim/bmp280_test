/*
    bmp280.c

    Created: 16/12/2020 4:27:01 PM
    Author: user
*/

#include <bmp280.h>

// Debug macro
//#define BMP280_DEBUG

// Structs defined
struct bmp280_calib_data bmp280_calib_data_read;

int32_t t_fine = 0;

/*
    This function verifies if sensor is initialized
    Assumes that SPI has been initialized already
*/
int bmp280_init(void) {

    //Check if read successfully from sensor ID
    uint8_t sensor_id = read8(BMP280_REGISTER_CHIPID);

    // Debugging
#ifdef BMP280_DEBUG
    printf("Sensor ID: 0x%x\r\n\r\n", sensor_id);
#endif

    if (sensor_id != BMP280_CHIPID) {
        return BMP280_INIT_ERR;
    }

    // Set default sampling
    bmp280_set_config(0, 4, 0); // 0.5 ms delay, 16x filter, 3-wire SPI
    bmp280_set_ctrl(2, 5, 3); // T oversample x2, P over x2, normal mode

    delay_ms(200);

    return BMP280_INIT_NO_ERR;
}

void bmp280_set_ctrl(uint8_t osrs_t, uint8_t osrs_p, uint8_t mode) {
    write8(BMP280_REGISTER_CONTROL, ((osrs_t & 0x7) << 5) | ((osrs_p & 0x7) << 2) | (mode & 0x3));
}

void bmp280_set_config(uint8_t t_sb, uint8_t filter, uint8_t spi3w_en) {
    write8(BMP280_REGISTER_CONTROL, ((t_sb & 0x7) << 5) | ((filter & 0x7) << 2) | (spi3w_en & 1));
}

/*!
     @brief  Returns the temperature from the sensor
     @returns the temperature read from the device in 4 digits (XX.YY = XXYY)
*/
int32_t bmp280_read_temperature(void) {
    int32_t var1, var2;
    int32_t adc_T = read24(BMP280_REGISTER_TEMPDATA);

#ifdef BMP280_DEBUG
    printf("adc: %ld\r\n", adc_T);
#endif

    adc_T = (adc_T >> 4);

#ifdef BMP280_DEBUG
    printf("adc: %ld\r\n", adc_T);
#endif

    // Calibrate the temperature sensor data
    var1 = ((((adc_T >> 3) - ((int32_t)bmp280_calib_data_read.dig_T1 << 1))) *
            ((int32_t)bmp280_calib_data_read.dig_T2)) >> 11;

    var2 = (((((adc_T >> 4) - ((int32_t)bmp280_calib_data_read.dig_T1)) * ((adc_T >> 4) - ((
                  int32_t)bmp280_calib_data_read.dig_T1))) >>
             12) * ((int32_t)bmp280_calib_data_read.dig_T3)) >> 14;

#ifdef BMP280_DEBUG
    printf("Prev t_fine: %ld\r\n", t_fine);
#endif

    t_fine = var1 + var2;

#ifdef BMP280_DEBUG
    printf("Curr t_fine: %ld\r\n", t_fine);
#endif

    int32_t T = (t_fine * 5 + 128) >> 8;

#ifdef BMP280_DEBUG
    printf("T: %ld\r\n", T);
#endif

    return T;
}

/*!
    Reads the barometric pressure from the device.
    @return Barometric pressure in Pa.
*/
int64_t bmp280_read_pressure(void) {
    int64_t var1, var2, p;

    // Must be done first to get the t_fine variable set up
    bmp280_read_temperature();

    int32_t adc_P = read24(BMP280_REGISTER_PRESSUREDATA);
    adc_P >>= 4;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t) bmp280_calib_data_read.dig_P6;
    var2 = var2 + ((var1 * (int64_t) bmp280_calib_data_read.dig_P5) << 17);
    var2 = var2 + (((int64_t) bmp280_calib_data_read.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t )bmp280_calib_data_read.dig_P3) >> 8) +
           ((var1 * (int64_t) bmp280_calib_data_read.dig_P2) << 12);
    var1 =
        (((((int64_t)1) << 47) + var1)) * ((int64_t) bmp280_calib_data_read.dig_P1) >> 33;

    if (var1 == 0) {
        return 0; // avoid exception caused by division by zero
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t) bmp280_calib_data_read.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t) bmp280_calib_data_read.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t) bmp280_calib_data_read.dig_P7) << 4);

    return p;
}

/*!
     @brief  Reads the factory-set coefficients
*/
void print_coefficients(void) {
    bmp280_calib_data_read.dig_T1 = read16_LE(BMP280_REGISTER_DIG_T1);
    bmp280_calib_data_read.dig_T2 = readS16_LE(BMP280_REGISTER_DIG_T2);
    bmp280_calib_data_read.dig_T3 = readS16_LE(BMP280_REGISTER_DIG_T3);

    //printf("\r\nbmp280_calib_data_read.dig_T1: %d\r\n", bmp280_calib_data_read.dig_T1);
    //printf("bmp280_calib_data_read.dig_T2: %d\r\n", bmp280_calib_data_read.dig_T2);
    //printf("bmp280_calib_data_read.dig_T3: %d\r\n\r\n", bmp280_calib_data_read.dig_T3);

    //bmp280_calib_data_read.dig_P1 = read16_LE(BMP280_REGISTER_DIG_P1);
    //bmp280_calib_data_read.dig_P2 = readS16_LE(BMP280_REGISTER_DIG_P2);
    //bmp280_calib_data_read.dig_P3 = readS16_LE(BMP280_REGISTER_DIG_P3);
    //bmp280_calib_data_read.dig_P4 = readS16_LE(BMP280_REGISTER_DIG_P4);
    //bmp280_calib_data_read.dig_P5 = readS16_LE(BMP280_REGISTER_DIG_P5);
    //bmp280_calib_data_read.dig_P6 = readS16_LE(BMP280_REGISTER_DIG_P6);
    //bmp280_calib_data_read.dig_P7 = readS16_LE(BMP280_REGISTER_DIG_P7);
    //bmp280_calib_data_read.dig_P8 = readS16_LE(BMP280_REGISTER_DIG_P8);
    //bmp280_calib_data_read.dig_P9 = readS16_LE(BMP280_REGISTER_DIG_P9);
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
    //spi_end_txn();

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
    value = (value << 8);
    value |= spixfer(0);
    value = (value << 8);
    value |= spixfer(0);

    // SS set to high - de-select slave
    ioport_set_pin_high(SPI_HARDWARE_SS);
    //spi_end_txn();

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






