# 4B25 - Embedded Systems for the Internet of Things
## Coursework 5

### Liam Self (lms213)

*Gonville and Caius College, CDT in Sensor Technologies*

## Overview

The purpose of this project was to create an embedded system based upon the KL03 board and Warp firmware to act as a health monitor for houseplants.

## Building and running the device
The same process is used to build and upload the software as is used by the underlying WARP firmware. Refer to Section 1 of https://github.com/physical-computation/Warp-firmware/blob/master/README.md for build instructions.

A wiring summary is submitted as a supplementary Excel file, which summizes the pins used on the KL03 to allow for reproduction.

## Summary of additions/changes

The files that have been amended or added for CW5, and an overview of the changes, are:

- src/boot/gpio_pins.h
    - Re-mapped pins

- src/boot/warp-kl03-ksdk1.1-boot.c
    - Removed unneeded functions and the Warp boot menu. testSensors() function performs reads of the sensors and outputs to OLED on a loop.

- src/boot/devSTEMMA.*
    - Driver for Adafruit STEMMA Soil Sensor

- src/boot/devSSD1331.*
    - Driver for SSD1331 OLED Display. Modified to provide helper functions for drawing pixels, lines and rectanges to display, and to provide basic fixed-size font support.

- src/boot/devSSD1331_FONT.h
    - Contains 6x8 bitmap for font.

- src/boot/devSI1445.*
    - Driver for light sensor

- src/boot/devAHT20.*
    - Driver for Temperature/Humidity sensor

- src/boot/CMakeLists.txt
    - Added new drivers, removed unneeded ones

- build/ksdk1.1/build.sh
    - Added new drivers, removed unnecessary drivers to/from build list


