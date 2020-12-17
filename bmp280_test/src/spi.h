/*
    spi.h

    Created: 16/12/2020 4:26:42 PM
    Author: user
*/

#include <ioport.h>

#define LSBFIRST 0
#define MSBFIRST 1

#define SPI_CLOCK_DIV4 0x00
#define SPI_CLOCK_DIV16 0x01
#define SPI_CLOCK_DIV64 0x02
#define SPI_CLOCK_DIV128 0x03
#define SPI_CLOCK_DIV2 0x04
#define SPI_CLOCK_DIV8 0x05
#define SPI_CLOCK_DIV32 0x06

#define SPI_MODE0 0x00
#define SPI_MODE1 0x04
#define SPI_MODE2 0x08
#define SPI_MODE3 0x0C

#define SPI_MODE_MASK 0x0C		// CPOL = bit 3, CPHA = bit 2 on SPCR
#define SPI_CLOCK_MASK 0x03		// SPR1 = bit 1, SPR0 = bit 0 on SPCR
#define SPI_2XCLOCK_MASK 0x01	// SPI2X = bit 0 on SPSR

// Hardware SPI Pins - ICSP
#define SPI_HARDWARE_MOSI		IOPORT_CREATE_PIN(PORTB, 3)
#define SPI_HARDWARE_MISO		IOPORT_CREATE_PIN(PORTB, 4)
#define SPI_HARDWARE_SCK		IOPORT_CREATE_PIN(PORTB, 5)
#define SPI_HARDWARE_SS			IOPORT_CREATE_PIN(PORTB, 2)		// Default uses digital pin 10

void spi_init(void);
uint8_t get_clock_divider_bits(uint8_t clock);
void set_clock_divider(uint8_t clockDiv);
void spi_begin_txn(uint32_t clock, uint8_t bit_order, uint8_t data_mode);
void spi_end_txn(void);