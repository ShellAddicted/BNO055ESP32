# BNO055ESP32 [![Build Status](https://travis-ci.org/ShellAddicted/BNO055ESP32.svg?branch=master)](https://travis-ci.org/ShellAddicted/BNO055ESP32)
This idf-component permits to control [Bosch-Sensortec's BNO055](https://www.bosch-sensortec.com/bst/products/all_products/bno055) using an [Espressif's ESP32 SoC](https://www.espressif.com/en/products/hardware/esp32/overview) (running [esp-idf](https://github.com/espressif/esp-idf)).

# Compatibility
Tested on ESP32D0WDQ6 (DevKitC) with [Adafruit's BNO055 Breakout Board](https://www.adafruit.com/product/2472) using UART protocol.

#### Supported Interfaces
- <b>UART</b> - fully Supported.
- <b>I²C</b> - <b>partially supported*</b>

*I²C is partially supported due to several issues probably caused by clock stretching.  
until espressif team will solve this issues, UART is suggested.

# Getting Started
***NOTE: this code is not (yet) Production Ready.***   
You can use this library as a component for your project: 
```
mkdir -p <YOUR_PROJECT_ROOT>/components/
cd <YOUR_PROJECT_ROOT>/components/
git clone https://github.com/ShellAddicted/BNO055ESP32.git
```
Remember to enable ```Compiler Options -> Enable C++ Exceptions``` using ```make menuconfig```

for more details see [examples/](https://github.com/ShellAddicted/BNO055ESP32/tree/master/examples)

## Wiring

### UART
PS1 -> 3.3v (HIGH) -> Enables UART protocol  
SCL -> UART RX (Default: GPIO_NUM_17)  
SDA -> UART TX (Default: GPIO_NUM_16)

### I2C
PS1 -> GND (LOW) -> Enables I²C protocol  
SCL -> SCL (Default: GPIO_NUM_22)  
SDA -> SDA (Default: GPIO_NUM_21)
