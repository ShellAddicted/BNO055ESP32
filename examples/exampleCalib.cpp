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
	BNO055 bno(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_16);
	try{
		bno.begin(); //BNO055 is in CONFIG_MODE until it is changed
		bno.enableExternalCrystal();
		//bno.setAxisRemap(BNO055_REMAP_CONFIG_P1, BNO055_REMAP_SIGN_P1); // see datasheet, section 3.4
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

	while (1){
		try{
			//Calibration 3 = fully calibrated, 0 = uncalibrated
			bno055_calibration_t cal = bno.getCalibration();
			bno055_vector_t v = bno.getVectorEuler();
			ESP_LOGI(TAG, "Euler: X: %.1f Y: %.1f Z: %.1f || Calibration SYS: %u GYRO: %u ACC:%u MAG:%u", v.x, v.y, v.z, cal.sys, cal.gyro, cal.accel, cal.mag);
			if (cal.gyro == 3 && cal.accel == 3 && cal.mag == 3){
				ESP_LOGI(TAG, "Fully Calibrated.");
				bno.setOprModeConfig(); //Change OPR_MODE
				bno055_offsets_t txt = bno.getSensorOffsets(); //NOTE: this must be executed in CONFIG_MODE
				ESP_LOGI(TAG, "\nOffsets:\nAccel: X:%d, Y:%d, Z:%d;\nMag: X:%d, Y:%d, Z:%d;\nGyro: X:%d, Y:%d, Z:%d;\nAccelRadius: %d;\nMagRadius: %d;\n", txt.accelOffsetX, txt.accelOffsetY, txt.accelOffsetZ, txt.magOffsetX, txt.magOffsetY, txt.magOffsetZ, txt.gyroOffsetX, txt.gyroOffsetY, txt.gyroOffsetZ, txt.accelRadius, txt.magRadius);
				ESP_LOGI(TAG,"Store this values, place them using setSensorOffsets() after every reset of the BNO055 to avoid the calibration process, unluckily MAG requires to be calibrated after every reset, for more information consult datasheet.");
				break;
			}
		}
		catch (BNO055BaseException& ex){
			ESP_LOGE(TAG, "Error: %s", ex.what());
			return;
		}
		catch (std::exception &ex){
			ESP_LOGE(TAG, "Error: %s", ex.what());
		}
		vTaskDelay(100 / portTICK_PERIOD_MS); //in fusion mode max output rate is 100hz (actual rate: 100ms (10hz))
	}
}
