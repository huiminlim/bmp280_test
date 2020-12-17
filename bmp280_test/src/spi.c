/*
    spi.c

    Created: 16/12/2020 4:26:53 PM
    Author: user
*/

#include <spi.h>
#include <ioport.h>
#include <delay.h>

int num_initialized = 0;

/*
    This function initializes and set up SPI
*/
void spi_init(void) {
    // Set Slave Select (SS) pin to High
    // De-select pin by default
    // NOTE: SS is pin 10 by default in hardware SPI now
    ioport_set_pin_high(SPI_HARDWARE_SS);

    // Set SS as output pin
    ioport_set_pin_dir(SPI_HARDWARE_SS, IOPORT_DIR_OUTPUT);

    // Initialize as SPI master
    SPCR |= (1 << MSTR);
    SPCR |= (1 << SPE);

    // Set SCK and MOSI as output pins
    // By doing this AFTER enabling SPI, we avoid accidentally
    // clocking in a single bit since the lines go directly
    // from "input" to SPI control.
    ioport_set_pin_dir(SPI_HARDWARE_SCK, IOPORT_DIR_OUTPUT);
    ioport_set_pin_dir(SPI_HARDWARE_MOSI, IOPORT_DIR_OUTPUT);

    // MISO pin is override to Input automatically
    //ioport_set_pin_dir(SPI_HARDWARE_MISO, IOPORT_DIR_INPUT);

    num_initialized = 0;
}

/*
    This function configures the SPI bus setting before the transaction
    Configuration must be done before calling any transfer/read functions
    Transaction set up assumes clock always in line with the standard SPI clock defined

    Example:
 		SPISettings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode)
 		SPISettings(500000, MSBFIRST, SPI_MODE0)
*/
void spi_begin_txn(uint32_t clock, uint8_t bit_order, uint8_t data_mode) {
    // Setup SPI in master, again
    SPCR |= (1 << SPE);
    SPCR |= (1 << MSTR);

    // Setup clock
    uint8_t clock_div_bits = get_clock_divider_bits(clock);
    set_clock_divider(clock_div_bits);

    // Setup data mode
    SPCR |= (data_mode & SPI_MODE_MASK);

    // Setup bit order
    SPCR |= ((bit_order == LSBFIRST) ? (1 << DORD) : 0);

    num_initialized++;
}

/*
    This function configures SPI bus to release it
*/
void spi_end_txn (void) {
    // Reduce the number of SPI initialized
    if (num_initialized) {
        num_initialized--;
    }

    // If no more usage of SPI bus, release the SPI bus
    // Disable SPI
    if (num_initialized == 0) {
        SPCR &= ~(1 << SPE);
    }
}

/*
    This function returns the clock div bits given a clock value for SPI

    Clock settings are defined as follows.
    Note that this shows SPI2X inverted, so the bits form increasing numbers.
    Also note that fosc/64 appears twice
    SPR1 SPR0 ~SPI2X Freq
     0    0     0   fosc/2
     0    0     1   fosc/4
     0    1     0   fosc/8
     0    1     1   fosc/16
     1    0     0   fosc/32
     1    0     1   fosc/64
     1    1     0   fosc/64
     1    1     1   fosc/128
*/
uint8_t get_clock_divider_bits(uint8_t clock) {
    uint8_t clockDiv;

    if (clock >= F_CPU / 2) {
        clockDiv = 0;
    }
    else if (clock >= F_CPU / 4) {
        clockDiv = 1;
    }
    else if (clock >= F_CPU / 8) {
        clockDiv = 2;
    }
    else if (clock >= F_CPU / 16) {
        clockDiv = 3;
    }
    else if (clock >= F_CPU / 32) {
        clockDiv = 4;
    }
    else if (clock >= F_CPU / 64) {
        clockDiv = 5;
    }
    else {
        clockDiv = 6;
    }

    // Compensate for the duplicate fosc/64
    if (clockDiv == 6) {
        clockDiv = 7;
    }

    return clockDiv;
}

/*
    This function sets the clock mode given a clock div value
*/
void set_clock_divider(uint8_t clockDiv) {
    // Invert the SPI2X bit
    uint8_t clock_div_invert = clockDiv ^ 0x1;

    SPCR |= ((clock_div_invert >> 1) & SPI_CLOCK_MASK);
    SPSR = (clock_div_invert & SPI_2XCLOCK_MASK);
}