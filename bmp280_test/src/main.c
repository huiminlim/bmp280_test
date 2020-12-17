#include <asf.h>
#include <spi.h>
#include <uart.h>
#include <bme280.h>

// Print all temp, pressure, altitude and humidity values
void print_all_values(void);

int main (void) {
    /* Insert system clock initialization code here (sysclk_init()). */
    board_init();

    ioport_init();
    uart_init();

    // Sensors initialization
    spi_init();

    printf("----- Default testing -----\r\n");
    uint8_t sensor_id = read8(BME280_REGISTER_CHIPID);
    printf("Sensor ID: 0x%x\r\n", sensor_id);

    if (sensor_id == 0x58) {
        int ret = bme280_init();

        if (ret == BME_INIT_NO_ERR) {
            printf("Sensor Initialized\r\n");
        }
        else {
            printf("Sensor initialization failed!\r\n");

            while (1);
        }

        while (1) {
            print_all_values();
            delay_ms(10000);
        }
    }

    else while (1);
}

void print_all_values(void) {
    char c[50]; //size of the number
    sprintf(c, "%f", bme280_read_temperature());
    printf("Temperature: %s\r\n", c);
}
