
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "BNO055ESP32.h"
static const char *TAG = "BNO055ESP32Example";

// This values are random, see exampleCalib.cpp for more details.
uint8_t offsets[] = {0xED, 0xFF, 0x2E, 0x00, 0x18, 0x00, 0x5B, 0xFF,  0x2C, 0xFE, 0xAD, 0xFF, 0xFF, 0xFF, 0xFE, 0xFF, 0x00 ,0x00 ,0xE8 ,0x03 ,0xEC ,0x02};

extern "C" void app_main(){
    BNO055 bno(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_16);
    double euler[3];
    try{
        bno.begin();
        bno.setExtCrystalUse(true);
        bno.setSensorOffsets(offsets);
        bno.setOpMode(BNO055_OPERATION_MODE_NDOF);
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
            bno.getVector(BNO055_VECTOR_EULER, euler);
            ESP_LOGI(TAG, "Euler: (%.1f;%.1f;%.1f) <--> (%d,%d,%d,%d)", (double)euler[0],(double)euler[1],(double)euler[2], (int)sys,(int)gyro,(int)accel,(int)mag);
        }
        catch (std::exception &ex){
            ESP_LOGE(TAG, "exc: %s", ex.what());
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); // in fusion mode output rate is 100hz
    }
}
