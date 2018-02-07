
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "BNO055ESP32.h"
static const char *TAG = "BNO055ESP32Example";

extern "C" void app_main(){
    BNO055 bno(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_16);
    try{
        bno.begin(); // at this point BNO055 is in CONFIG_MODE until it is changed
        bno.setExtCrystalUse(true);
        //HOW TO RESTORE OFFSETS (EXAMPLE VALUES):
        // bno055_offsets_t ofx;
        // ofx.accelOffsetX = 29;
        // ofx.accelOffsetY = 24;
        // ofx.accelOffsetZ = 16;
        // ofx.magOffsetX = -243;
        // ofx.magOffsetY = -420;
        // ofx.magOffsetZ = -131;
        // ofx.gyroOffsetX = 1;
        // ofx.gyroOffsetY = -1;
        // ofx.gyroOffsetZ = 0;
        // ofx.accelRadius = 0;
        // ofx.magRadius = 662;
        // bno.setSensorOffsets(ofx); //NOTE: this must be executed in CONFIG_MODE
        bno.setOpMode(BNO055_OPERATION_MODE_NDOF); // Change OPR_MODE
        ESP_LOGI(TAG, "Setup Done.");
    }
    catch (std::exception& ex){
        ESP_LOGE(TAG, "Setup Failed, Error: %s", ex.what());
        return;
    }

    while (1){
        try{
            uint8_t sys, gyro, accel, mag;
            bno.getCalibration(&sys,&gyro,&accel,&mag);
            bno055_vector_t v = bno.getVectorEuler();
            ESP_LOGI(TAG, "Euler:(%.1f;%.1f;%.1f) <--> (%d,%d,%d,%d)",(double)v.x, (double)v.y, (double)v.z, (int)sys, (int)gyro, (int)accel, (int)mag);
            if (gyro == 3 && accel == 3 && mag == 3){
                ESP_LOGI(TAG, "Fully Calibrated.");
                bno.setOpMode(BNO055_OPERATION_MODE_CONFIG); //Change OPR_MODE
                bno055_offsets_t txt = bno.getSensorOffsets(); //NOTE: this must be executed in CONFIG_MODE
                ESP_LOGI(TAG, "\nOffsets:\nAccel: X:%d, Y:%d, Z:%d;\nMag: X:%d, Y:%d, Z:%d;\nGyro: X:%d, Y:%d, Z:%d;\nAccelRadius: %d;\nMagRadius: %d;\n", txt.accelOffsetX, txt.accelOffsetY, txt.accelOffsetZ, txt.magOffsetX, txt.magOffsetY, txt.magOffsetZ, txt.gyroOffsetX, txt.gyroOffsetY, txt.gyroOffsetZ, txt.accelRadius, txt.magRadius);
                break;
            }
        }
        catch (std::exception &ex){
            ESP_LOGE(TAG, "exc: %s", ex.what());
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); // in fusion mode output rate is 100hz
    }
}
