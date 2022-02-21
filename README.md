# BNO055ESP32
C++ Interface for the [Bosch-Sensortec's BNO055](https://www.bosch-sensortec.com/bst/products/all_products/bno055)
compatible with [Espressif's ESP32 SoC](https://www.espressif.com/en/products/hardware/esp32/overview) running [esp-idf](https://github.com/espressif/esp-idf).

# Compatibility
Tested on ESP32D0WDQ6 (DevKitC) with [Adafruit's BNO055 Breakout Board](https://www.adafruit.com/product/4646)

## Supported Interfaces

| Interface | Notes                |
|-----------|----------------------|
| UART      | Fully Supported      |
| I²C       | Partially Supported* |

*unstable (due to clock stretching)

# Getting Started
***NOTE: this code is not (yet) Production Ready.***   

You can use this as a [managed-component](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html) for your project by adding the following to your `idf_component.yml`:
```yaml
BNO055ESP32:
  path: .
  git: https://github.com/ShellAddicted/BNO055ESP32.git
```

Alternatively, you can use this as a component for your project: 
```bash
mkdir components/
cd components/
git clone https://github.com/ShellAddicted/BNO055ESP32.git
```
Remember to enable ```Compiler Options -> Enable C++ Exceptions``` using ```idf.py menuconfig```.

For more details see [examples/](https://github.com/ShellAddicted/BNO055ESP32/tree/master/examples)


# Wiring

| IMU Pin | UART                           | I²C                        |
|---------|--------------------------------|----------------------------|
| PS1     | 3.3v                           | GND                        |
| SCL     | UART RX (Default: GPIO_NUM_17) | SCL (Default: GPIO_NUM_22) |
| SDA     | UART TX (Default: GPIO_NUM_16) | SDA (Default: GPIO_NUM_21) |
