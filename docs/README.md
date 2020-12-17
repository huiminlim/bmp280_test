# Interfacing BMP280 sensor in C

This repo contains test code to verify porting of Arduino's SPI and Adafruit's BMP280 C++ library to C.

Development was done in Atmel Studio 7.0 and tested on an Arduino Uno with ATmega328P microcontroller.

This is part of FYP project to use Bosch's BMP280 sensor in C.

## Credits

Libraries are adapted from:

* [Arduino SPI library](https://github.com/arduino/ArduinoCore-avr/tree/master/libraries/SPI)

* [Adafruit's BMP280 library](https://github.com/adafruit/Adafruit_BMP280_Library)

Credits goes to them.

## BMP280

Bosch's BMP280 humidity sensor is used and the datasheet can be found [here](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/pressure-sensors-bmp280-1.html).

## Dependencies

Development was done in Atmel Studio 7.0 and certain Atmel Software Framework (ASF) libraries were used.

They include:

* [Delay Routine](https://asf.microchip.com/docs/latest/saml21/html/group__group__common__services__delay.html)

* [IOPORT Service](https://asf.microchip.com/docs/latest/saml21/html/group__ioport__group.html#gabc09edad7c3187dec63ce47e6f1b3c51)

## Expected Output

Currently, only the temperature reading can be obtained as the FYP only requires the temperature reading.

The output should look like this:

![Capture.PNG][Expected output]