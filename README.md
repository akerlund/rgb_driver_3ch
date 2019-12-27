# LED driver PCB, 3ch RGB

![Version Three](https://github.com/akerlund/rgb_driver_3ch/blob/master/kicad/version_3.png)

STM32F100 microcontroller with small SOT23 MOSFETs to drive 12V LED strips.
Four LEDs and two buttons for custom use.
Three potentiometers for controlling the light; HSL, i.e., Hue, Saturation and Light.
A CP2102 serial to USB interface.
Pinout for other peripherals, i.e., I2C, SPI and UART.

## Build
```
cd code
make
```

## Program
I use Devilholk's verion of [stm32flash](https://github.com/devilholk/stm32flash),
with a cheap CP2102 USB to UART Bridge Controller.

### Programming sequence example

```
~/rgb_driver_3ch$ stm32flash -b 115200 -Rcs.rts -Bcs.dtr -w code/build/driver_rgb_3ch.bin /dev/ttyUSB0
stm32flash - http://stm32flash.googlecode.com/

Using Parser : Raw BINARY
Serial Config: 115200 8E1
Version      : 0x22
Option 1     : 0x00
Option 2     : 0x00
Device ID    : 0x0420 (Medium-density VL)
RAM          : 8KiB  (512b reserved by bootloader)
Flash        : 128KiB (sector size: 4x1024)
Option RAM   : 15b
System RAM   : 2KiB

Wrote address 0x08003304 (100.00%) Done.
```


### Pin configurations
Make sure you connect cables like this;

PCB | CP2102
-- | --
3.3V | N/C
GND | GND
TX | RXI
RX | TXO
RESET | RTS
BOOT | DTR


If you haven't, remember to add your username to the dialout group

```
sudo adduser <user_name> dialout
```