
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "BNO055ESP32.h"
static const char *TAG = "BNO055ESP32Example";

extern "C" void app_main(){
    BNO055 bno(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_16);
    try{
        bno.begin();
        bno.setExtCrystalUse(true);
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
            bno055_vector_t v = bno.getVectorEuler();
            ESP_LOGI(TAG, "Euler: (%.1f;%.1f;%.1f) <--> (%d,%d,%d,%d)", (double)v.x,(double)v.y,(double)v.z, (int)sys,(int)gyro,(int)accel,(int)mag);
        }
        catch (std::exception &ex){
            ESP_LOGE(TAG, "exc: %s", ex.what());
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); // in fusion mode output rate is 100hz
    }
}
