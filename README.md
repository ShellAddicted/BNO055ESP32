# Getting Started
***NOTE: this code is not (yet) Production Ready.***   
You can use this library as a component for your project:   
```
mkdir -p <YOUR_PROJECT_ROOT>/components/
cd <YOUR_PROJECT_ROOT>/components/
git clone https://github.com/ShellAddicted/BNO055ESP32.git
```
Remember to enable ```Compiler Options -> Enable C++ Exceptions``` using ```make menuconfig```

this library uses the BNO055's UART interface due to issues with i2c probabily caused by Clock Stretching.
when these issues will fixed, i2c support will be added.

Tested on ESP32-DevKitC & Adafruit's BNO055 Breakout Board.
