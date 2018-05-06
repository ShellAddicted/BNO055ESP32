# Description
This library permits to control Bosch-Sensortec's BNO055 with an ESP32 SoC (esp-idf)

# Compatibility
Tested on ESP32D0WDQ6 (DevKitC) with [Adafruit's BNO055 Breakout Board](https://www.adafruit.com/product/2472) using UART protocol.

<b>I²C</b> is not supported due to issues probably caused by clock stretching.  
when this issues will be fixed I²C will be supported.

# Getting Started
***NOTE: this code is not (yet) Production Ready.***   
You can use this library as a component for your project: 
```
mkdir -p <YOUR_PROJECT_ROOT>/components/
cd <YOUR_PROJECT_ROOT>/components/
git clone https://github.com/ShellAddicted/BNO055ESP32.git
```
Remember to enable ```Compiler Options -> Enable C++ Exceptions``` using ```make menuconfig```

and see [examples/](https://github.com/ShellAddicted/BNO055ESP32/tree/master/examples)

## Wiring

PS1 -> 3.3v (HIGH) -> Enables UART protocol instead of I²C  
SDA -> UART RX  
SCL -> UART TX  
obviously, BNO055 and ESP32 SoC <b>must</b> have a common ground. 