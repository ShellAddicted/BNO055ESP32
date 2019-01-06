// MIT License

// Copyright (c) 2019 ShellAddicted <github.com/ShellAddicted>

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/*
        https://www.bosch-sensortec.com/bst/products/all_products/bno055
        Reference Datasheet: BST_BNO055_DS000_14 (consulted in January 2018)
*/
#include "BNO055ESP32.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
static const char* TAG = "BNO055ESP32Example";

extern "C" void app_main() {
    // see exampleCalibration.cpp for more details.
    // bno055_offsets_t storedOffsets;
    // storedOffsets.accelOffsetX = 29;
    // storedOffsets.accelOffsetY = 24;
    // storedOffsets.accelOffsetZ = 16;
    // storedOffsets.magOffsetX = -243;
    // storedOffsets.magOffsetY = -420;
    // storedOffsets.magOffsetZ = -131;
    // storedOffsets.gyroOffsetX = 1;
    // storedOffsets.gyroOffsetY = -1;
    // storedOffsets.gyroOffsetZ = 0;
    // storedOffsets.accelRadius = 0;
    // storedOffsets.magRadius = 662;

    /* to use I2C uncomment this block and remove line 66

    // Setup I²C
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = GPIO_NUM_21;
    conf.scl_io_num = GPIO_NUM_22;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    i2c_set_timeout(I2C_NUM_0, 30000);

    //to use i²C leave the following line active
BNO055 bno((i2c_port_t)I2C_NUM_0, 0x28); // BNO055 I2C Addr can be 0x28 or 0x29 (depends on your hardware)
    */

    // to use UART use the following line active (UART is suggested)
    BNO055 bno(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_16);

    try {
        bno.begin();  // BNO055 is in CONFIG_MODE until it is changed
        bno.enableExternalCrystal();
        // bno.setSensorOffsets(storedOffsets);
        // bno.setAxisRemap(BNO055_REMAP_CONFIG_P1, BNO055_REMAP_SIGN_P1); // see datasheet, section 3.4
        /* you can specify a PoWeRMode using:
                - setPwrModeNormal(); (Default on startup)
                - setPwrModeLowPower();
                - setPwrModeSuspend(); (while suspended bno055 must remain in CONFIG_MODE)
        */
        bno.setOprModeNdof();
        ESP_LOGI(TAG, "Setup Done.");
    } catch (BNO055BaseException& ex) {  // see BNO055ESP32.h for more details about exceptions
        ESP_LOGE(TAG, "Setup Failed, Error: %s", ex.what());
        return;
    } catch (std::exception& ex) {
        ESP_LOGE(TAG, "Setup Failed, Error: %s", ex.what());
        return;
    }

    try {
        int8_t temperature = bno.getTemp();
        ESP_LOGI(TAG, "TEMP: %d°C", temperature);

        int16_t sw = bno.getSWRevision();
        uint8_t bl_rev = bno.getBootloaderRevision();
        ESP_LOGI(TAG, "SW rev: %d, bootloader rev: %u", sw, bl_rev);

        bno055_self_test_result_t res = bno.getSelfTestResult();
        ESP_LOGI(TAG, "Self-Test Results: MCU: %u, GYR:%u, MAG:%u, ACC: %u", res.mcuState, res.gyrState, res.magState,
                 res.accState);
    } catch (BNO055BaseException& ex) {  // see BNO055ESP32.h for more details about exceptions
        ESP_LOGE(TAG, "Something bad happened: %s", ex.what());
        return;
    } catch (std::exception& ex) {
        ESP_LOGE(TAG, "Something bad happened: %s", ex.what());
        return;
    }

    while (1) {
        try {
            // Calibration 3 = fully calibrated, 0 = not calibrated
            bno055_calibration_t cal = bno.getCalibration();
            bno055_vector_t v = bno.getVectorEuler();
            ESP_LOGI(TAG, "Euler: X: %.1f Y: %.1f Z: %.1f || Calibration SYS: %u GYRO: %u ACC:%u MAG:%u", v.x, v.y, v.z, cal.sys,
                     cal.gyro, cal.accel, cal.mag);
            if (cal.gyro == 3 && cal.accel == 3 && cal.mag == 3) {
                ESP_LOGI(TAG, "Fully Calibrated.");
                bno.setOprModeConfig();                         // Change to OPR_MODE
                bno055_offsets_t txt = bno.getSensorOffsets();  // NOTE: this must be executed in CONFIG_MODE
                ESP_LOGI(TAG,
                         "\nOffsets:\nAccel: X:%d, Y:%d, Z:%d;\nMag: X:%d, Y:%d, Z:%d;\nGyro: X:%d, Y:%d, Z:%d;\nAccelRadius: "
                         "%d;\nMagRadius: %d;\n",
                         txt.accelOffsetX, txt.accelOffsetY, txt.accelOffsetZ, txt.magOffsetX, txt.magOffsetY, txt.magOffsetZ,
                         txt.gyroOffsetX, txt.gyroOffsetY, txt.gyroOffsetZ, txt.accelRadius, txt.magRadius);
                ESP_LOGI(TAG,
                         "Store this values, place them using setSensorOffsets() after every reset of the BNO055 to avoid the "
                         "calibration process, unluckily MAG requires to be calibrated after every reset, for more information "
                         "check datasheet.");
                break;
            }
        } catch (BNO055BaseException& ex) {
            ESP_LOGE(TAG, "Something bad happened: %s", ex.what());
            return;
        } catch (std::exception& ex) {
            ESP_LOGE(TAG, "Something bad happened: %s", ex.what());
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);  // in fusion mode max output rate is 100hz (actual rate: 100ms (10hz))
    }
    /* to [forcefully] stop the communication, set BNO055 in PWR_MODE_SUSPEND, and free all the allocated resources you can use:

            bno.stop(); // (if you can use something it does NOT mean you should!!!!)

            in most cases (99.9%) you don't have to care about stop() just don't use it,
            use it only when NECESSARY otherwise destructor ~BNO055() will 'autonomously' take care of everything.

            DO NOT USE stop() to disable bno055 for short periods because it's inefficient,
            see setPwrMode() and setOprMode*() functions and datasheet to do that in the right way.
    */
}
