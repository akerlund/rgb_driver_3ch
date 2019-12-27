# LED driver PCB, 3ch RGB

![Image of Yaktocat](https://github.com/akerlund/rgb_driver_3ch/blob/master/kicad/version_3.png)

STM32F100 microcontroller with small SOT23 MOSFET to drive 12V LED strips.
Four LEDs and two buttons for custom use.
Three potentiometers for HSL; Hue, Saturation and Light.
A CP2102 serial to USB interface.
Pinout for other peripherals, i.e., I2C, SPI and UART.

## Build and program
cd code
make
stm32flash -b 115200 -Rcs.rts -Bcs.dtr -w ./build/driver_rgb_3ch.bin /dev/ttyUSB0
