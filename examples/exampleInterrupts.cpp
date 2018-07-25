// MIT License

// Copyright (c) 2018 ShellAddicted <github.com/ShellAddicted>

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
	For more info: https://www.bosch-sensortec.com/bst/products/all_products/bno055
	Information in this library refers to BST_BNO055_DS000_14 (Consulted in January 2018)
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "BNO055ESP32.h"
static const char *TAG = "BNO055ESP32Example";

extern "C" void app_main(){
	// This values are random, see exampleCalibration.cpp for more details.
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

	BNO055 bno(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_16, GPIO_NUM_MAX, GPIO_NUM_23); // GPIO_NUM_MAX means unset
	try{
		bno.begin(); //BNO055 is in CONFIG_MODE until it is changed
		bno.enableExternalCrystal();
		//bno.setSensorOffsets(storedOffsets);
		//bno.setAxisRemap(BNO055_REMAP_CONFIG_P1, BNO055_REMAP_SIGN_P1); // see datasheet, section 3.4
		bno.setAccelAnyMotionInterrupt(2, 2, true, true, true); // configure the interrupt, see datasheet for more details.
		bno.setAccelNoMotionInterrupt(0,0, true, true, true);
		bno.enableAccelAnyMotionInterrupt(true); // you can disable it with disableAccelAnyMotionInterrupt();
		bno.enableAccelNoMotionInterrupt(true);
		bno.setOprModeNdof();
		ESP_LOGI(TAG, "Setup Done.");
	}
	catch (BNO055BaseException& ex){
		ESP_LOGE(TAG, "Setup Failed, Error: %s", ex.what());
		return;
	}
	catch (std::exception& ex){
		ESP_LOGE(TAG, "Setup Failed, Error: %s", ex.what());
		return;
	}

	int8_t temperature = bno.getTemp();
	ESP_LOGI(TAG, "TEMP: %d°C", temperature);
	
	while (1){
		if (bno.interruptFlag == true){
			//See bno055_interrupts_status_t for more details.
			bno055_interrupts_status_t ist = bno.getInterruptsStatus();
			// remmeber that multiple interrupts can be triggered at the same time. so you shouldn't use 'else if'
			if (ist.accelAnyMotion == 1){
				ESP_LOGI(TAG, "AccelAnyMotion Interrupt received.");
			}
			if (ist.accelNoSlowMotion == 1){
				ESP_LOGI(TAG, "accelNoSlowMotion Interrupt received.");
			}
			bno.clearInterruptPin(); //don't forget to place this.
		}
		try{
			//Calibration 3 = fully calibrated, 0 = uncalibrated
			bno055_calibration_t cal = bno.getCalibration();
			bno055_vector_t v = bno.getVectorEuler();
			ESP_LOGI(TAG, "Euler: X: %.1f Y: %.1f Z: %.1f || Calibration SYS: %u GYRO: %u ACC:%u MAG:%u", v.x, v.y, v.z, cal.sys, cal.gyro, cal.accel, cal.mag);
		}
		catch (BNO055BaseException& ex){
			ESP_LOGE(TAG, "Error: %s", ex.what());
			return;
		}
		catch (std::exception &ex){
			ESP_LOGE(TAG, "Error: %s", ex.what());
		}
		vTaskDelay(100 / portTICK_PERIOD_MS); // in fusion mode max output rate is 100hz (actual rate: 100ms (10hz))
	}
}
