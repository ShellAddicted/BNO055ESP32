/* exampleQuaternion.cpp
 * Copyright 2018 ShellAddicted <shelladdicted@gmail.com>
 * GitHub: https://github.com/ShellAddicted
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; If not, see <http://www.gnu.org/licenses/>.
*/

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

	BNO055 bno(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_16);
	try{
		bno.begin(); //BNO055 is in CONFIG_MODE until it is changed
		bno.setExtCrystalUse(true);
		//bno.setSensorOffsets(storedOffsets);
		//bno.setAxisRemap(BNO055_REMAP_CONFIG_P1, BNO055_REMAP_SIGN_P1); // see datasheet, section 3.4
		bno.setOpMode(BNO055_OPERATION_MODE_NDOF);
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
			bno055_calibration_t cal = bno.getCalibration();
			bno055_vector_t v = bno.getQuaternion();
			ESP_LOGI(TAG, "Quaternion: W: %1.f X: %.1f Y: %.1f Z: %.1f || Calibration SYS: %u GYRO: %u ACC:%u MAG:%u", v.w,v.x, v.y, v.z, cal.sys, cal.gyro, cal.accel, cal.mag);
		}
		catch (BNO055BaseException& ex){
			ESP_LOGE(TAG, "Error: %s", ex.what());
			return;
		}
		catch (std::exception &ex){
			ESP_LOGE(TAG, "Error: %s", ex.what());
		}
		vTaskDelay(100 / portTICK_PERIOD_MS); // in fusion mode output rate is 100hz
	}
}
