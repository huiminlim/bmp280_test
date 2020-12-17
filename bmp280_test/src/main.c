#include <asf.h>
#include <spi.h>
#include <uart.h>
#include <bmp280.h>

// Print all temp, pressure, altitude and humidity values
void print_all_values(void);

int main (void) {
    /* Insert system clock initialization code here (sysclk_init()). */
    board_init();

    ioport_init();
    uart_init();

    // Sensors initialization
    spi_init();

    delay_ms(1000);

    printf("----- Default testing -----\r\n");
    //uint8_t sensor_id = read8(BMP280_REGISTER_CHIPID);
    //printf("Sensor ID: 0x%x\r\n", sensor_id);

    //delay_ms(1000);

    int ret = bmp280_init();

    if (ret == BMP280_INIT_NO_ERR) {
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

void print_all_values(void) {
    char str[6];
    int32_t temp = bmp280_read_temperature();
    snprintf(str, sizeof(str), "%ld.%ld", (temp - temp % 100) / 100, temp % 100);
    printf("Temperature: %s\r\n", str);
}
