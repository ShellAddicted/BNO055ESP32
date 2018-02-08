/* BNO055ESP32.cpp
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

#include "BNO055ESP32.h"

/* used in ESP_LOG macros */
static const char *BNO055_LOG_TAG = "BNO055";

static void IRAM_ATTR bno055_interrupt_handler(void* arg){
    //static_cast<BNO055*>(arg)->something;
}

BNO055::BNO055(uart_port_t uartPort, gpio_num_t txPin, gpio_num_t rxPin, gpio_num_t rstPin, gpio_num_t intPin){
    _uartPort = uartPort;
    _txPin = txPin;
    _rxPin = rxPin;
    _rstPin = rstPin;
    _intPin = intPin;
}

std::exception BNO055::getException(uint8_t errcode){
    if (errcode == 0x02){
        return BNO055ReadFail();
    }
    else if (errcode == 0x03){
        return BNO055WriteFail();
    }
    else if (errcode == 0x04){
        return BNO055RegmapInvalidAddress();
    }
    else if (errcode == 0x05){
        return BNO055RegmapWriteDisabled();
    }
    else if (errcode == 0x06){
        return BNO055WrongStartByte();
    }
    else if (errcode == 0x07){
        return BNO055BusOverRunError();
    }
    else if (errcode == 0x08){
        return BNO055MaxLengthError();
    }
    else if (errcode == 0x09){
        return BNO055MinLengthError();
    }
    else if (errcode == 0x0A){
        return BNO055ReceiveCharacterTimeout();
    }
    else{
        return BNO055UnknowError();
    }
}

void BNO055::readLen(bno055_reg_t reg, uint8_t len, uint8_t *buffer, uint32_t timoutMS){
    uint8_t res = 0;
    memset(buffer, 0, len);
    
    uint8_t *cmd = (uint8_t*) malloc(4);
    cmd[0] = 0xAA; // Start Byte
    cmd[1] = 0x01; // Read
    cmd[2] = reg & 0xFF;
    cmd[3] = len & 0xFF; // len in bytes

    uint8_t *data = (uint8_t *) malloc(len+2);

    for (int round = 1; round <= UART_ROUND_NUM; round++){
        #ifndef BNO055_DEBUG_OFF
        ESP_LOGD(BNO055_LOG_TAG, "(RL) Round %d", round);
        #endif

        //Send command over UART
        uart_flush(_uartPort);
        uart_write_bytes(_uartPort, (const char*) cmd, 4);

        #ifndef BNO055_DEBUG_OFF
        ESP_LOG_BUFFER_HEXDUMP(BNO055_LOG_TAG, (const char*) cmd, 4, ESP_LOG_DEBUG);
        #endif

        if (timoutMS <= 0){
            free(cmd);
            free(data);
            return; // Do no expect ACK
        }
        // else expect ACK
        
        // Read data from the UART
        int rxBytes = uart_read_bytes(_uartPort, data, (len+2), timoutMS / portTICK_RATE_MS);
        if (rxBytes > 0) {
            //data[rxBytes] = 0;

            #ifndef BNO055_DEBUG_OFF
            ESP_LOGD(BNO055_LOG_TAG, "(RL) Read %d bytes", rxBytes);
            ESP_LOG_BUFFER_HEXDUMP(BNO055_LOG_TAG, data, rxBytes, ESP_LOG_DEBUG);
            #endif

            res = data[1]; // in case of error, this will be the errorCode.

            if (round == UART_ROUND_NUM){
                free(cmd);
                free(data);
                throw getException(res);
            }
            
            else if (data[0] == 0xBB){ // OK
                memcpy(buffer, data+2, len); // remove header bytes & Write back response
                free(cmd);
                free(data);
                break;
            }

            else if (data[0] == 0xEE){ //Error
                if (data[1] == 0x07 || data[1] == 0x02 || data[1] == 0x0A){
                    //ESP_LOGE(BNO055_LOG_TAG, "(RL) Error: %d, Resending command...", res);
                    continue;
                }
                ESP_LOGE(BNO055_LOG_TAG, "(RL) Error: %d", res);
                free(cmd);
                free(data);
                throw getException(res);
            }

            else{
                ESP_LOGE(BNO055_LOG_TAG, "(RL) Error: %d (BNO55_UNKNOW_ERROR)", (int)res);
                free(cmd);
                free(data);             
                throw getException(res);
            }
        }
        else{
            free(cmd);
            free(data);
            throw BNO055UartTimeout();
        }
    }
}

void BNO055::writeLen(bno055_reg_t reg, uint8_t *data2write, uint8_t len, uint32_t timoutMS){
    uint8_t res = 0;

    uint8_t *cmd = (uint8_t *) malloc(len+4);
    cmd[0] = 0xAA; // Start Byte
    cmd[1] = 0x00; // Write
    cmd[2] = reg & 0xFF;
    cmd[3] = len & 0xFF; // len in bytes
    memcpy(cmd+4, data2write, len);
    
    uint8_t *data = (uint8_t *) malloc(2);

    // Read data from the UART
    for (int round = 1; round <= UART_ROUND_NUM; round++){

        #ifndef BNO055_DEBUG_OFF
        ESP_LOGD(BNO055_LOG_TAG, "(WL) Round %d", round); //DEBUG
        #endif

        //SEND
        uart_flush(_uartPort);
        uart_write_bytes(_uartPort, (const char*) cmd,(len+4));

        #ifndef BNO055_DEBUG_OFF
        ESP_LOG_BUFFER_HEXDUMP(BNO055_LOG_TAG, (const char*) cmd,(len+4), ESP_LOG_DEBUG);
        #endif

        if (timoutMS <= 0){
            free(cmd);
            free(data);
            return; // do not expect ACK
        }
        //else expect ACK
        
        int rxBytes = uart_read_bytes(_uartPort, data, 2, timoutMS / portTICK_RATE_MS);
        if (rxBytes > 0) {
            res = data[1]; // in case of error, this will be the errorCode.
            //data[rxBytes] = 0;
            
            #ifndef BNO055_DEBUG_OFF
            ESP_LOGD(BNO055_LOG_TAG, "(WL) Read %d bytes", rxBytes); //DEBUG
            ESP_LOG_BUFFER_HEXDUMP(BNO055_LOG_TAG, (const char*) data,rxBytes, ESP_LOG_DEBUG);
            #endif

            if (round == UART_ROUND_NUM){
                free(cmd);
                free(data);
                throw getException(res);
            }

            else if (data[0] == 0xEE){
                if (data[1] == 0x07 || data[1] == 0x03 || data[1] == 0x06|| data[1] == 0x0A){
                    //ESP_LOGE(BNO055_LOG_TAG, "(WL) Error: %d, resending command...", (int)data[1]);
                    continue;
                }
                free(cmd);
                free(data);
                if (res == 0x01){ // OK
                    break;
                }
                else{
                    ESP_LOGE(BNO055_LOG_TAG, "(WL) Error: %d.", (int)res);
                    throw getException(res);
                }
            }

            else{
                free(cmd);
                free(data);
                ESP_LOGE(BNO055_LOG_TAG, "(WL) Error: %d (BNO55_UNKNOW_ERROR)", (int)res);
                throw getException(res);
            }
        }
        else{
            free(cmd);
            free(data);
            throw BNO055UartTimeout();
        }
    }
}

void BNO055::read8(bno055_reg_t reg, uint8_t *val, uint32_t timoutMS){
    readLen(reg, 1, val, timoutMS);
}

void BNO055::write8(bno055_reg_t reg, uint8_t val, uint32_t timoutMS){
    writeLen(reg, &val, 1, timoutMS);
}

void BNO055::setPage(uint8_t page, bool forced){
    if (_page != page || forced == true){
        write8(BNO055_REG_PAGE_ID, page);
        _page = page;
    }
}

void BNO055::setOpMode(bno055_opmode_t mode, bool forced){
    setPage(0);
    if (_mode != mode || forced == true){
        write8(BNO055_REG_OPR_MODE, mode);
        vTaskDelay(30 / portTICK_PERIOD_MS);
        _mode = mode;
    }
}

void BNO055::setPwrMode(bno055_powermode_t pwrMode){
    setPage(0);
    write8(BNO055_REG_PWR_MODE, pwrMode);
}

void BNO055::setExtCrystalUse(bool state){
    setPage(0);
    if (state == true) {
        write8(BNO055_REG_SYS_TRIGGER, 0x80);
    }
    else {
        write8(BNO055_REG_SYS_TRIGGER, 0x00);
    }
    vTaskDelay(650 /portTICK_PERIOD_MS);
}

bno055_system_status_t BNO055::getSystemStatus(){
    setPage(0);
    uint8_t tmp;
    read8(BNO055_REG_SYS_STATUS, &tmp);
    return (bno055_system_status_t)tmp;
}

bno055_self_test_result_t BNO055::getSelfTestResult(){
    //1 = test passed <-> 0 = test failed
    setPage(0);
    uint8_t tmp;
    bno055_self_test_result_t res;
    read8(BNO055_REG_ST_RESULT, &tmp);
    res.mcuState = (tmp >> 3) & 0x01;
    res.gyrState = (tmp >> 2) & 0x01;
    res.magState = (tmp >> 1) & 0x01;
    return res;
}

bno055_system_error_t BNO055::getSystemError(){
    setPage(0);
    uint8_t tmp;
    read8(BNO055_REG_SYS_ERR, &tmp);
    return (bno055_system_error_t)tmp;
}

bno055_calibration_t BNO055::getCalibration(){
    setPage(0);
    bno055_calibration_t cal;
    uint8_t calData = 0;
    read8(BNO055_REG_CALIB_STAT, &calData);
    cal.sys = (calData >> 6) & 0x03;
    cal.gyro = (calData >> 4) & 0x03;
    cal.accel = (calData >> 2) & 0x03;
    cal.mag = calData & 0x03;
    return cal;
}

int8_t BNO055::getTemp(){
    setPage(0);
    uint8_t t;
    read8(BNO055_REG_TEMP, &t);
    t*=tempScale;
    return t;
}

void BNO055::reset(){
    if (_rstPin == GPIO_NUM_MAX){
        #ifndef BNO055_DEBUG_OFF
        ESP_LOGD(BNO055_LOG_TAG, "RST -> using serial bus"); //DEBUG
        #endif
        write8(BNO055_REG_SYS_TRIGGER, 0x20, 0); //RST (0 timeout because RST is not Acknoledged)
    }
    else{
        #ifndef BNO055_DEBUG_OFF
        ESP_LOGD(BNO055_LOG_TAG, "RST -> using hardware pin"); //DEBUG
        #endif
        gpio_pad_select_gpio(_rstPin);
	    gpio_set_direction(_rstPin, GPIO_MODE_OUTPUT);
        gpio_set_level(_rstPin, 0); // turn OFF
        vTaskDelay(1 / portTICK_PERIOD_MS);
        gpio_set_level(_rstPin, 1); // turn ON
    }
    vTaskDelay(700 / portTICK_PERIOD_MS); // (RE)BOOT TIME (datasheet raccomands 650ms)
}

bno055_vector_t BNO055::getVector(bno055_vector_type_t vec){
    setPage(0);
    uint8_t *buffer = (uint8_t*) malloc(6);
  
    /* Read (6 bytes) */
    try{
        readLen((bno055_reg_t)vec, 6, buffer);
    }
    catch(std::exception exc){
        free(buffer);
        throw exc;
    }

    double scale = 1;

    if (vec == BNO055_VECTOR_MAGNETOMETER){
        scale = magScale;
    }

    else if (vec == BNO055_VECTOR_ACCELEROMETER || vec == BNO055_VECTOR_LINEARACCEL || vec == BNO055_VECTOR_GRAVITY){
        scale = accelScale;
    }

    else if (vec == BNO055_VECTOR_GYROSCOPE){
        scale = angularRateScale;
    }

    else if (vec == BNO055_VECTOR_EULER){
        scale = eulerScale;
    }
    
    int16_t x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
    int16_t y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
    int16_t z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);
    free(buffer);
    
    bno055_vector_t xyz;
    xyz.x = (double)x/scale;
    xyz.y = (double)y/scale;
    xyz.z = (double)z/scale;
    return xyz;
}

bno055_vector_t BNO055::getVectorAccelerometer(){
    return getVector(BNO055_VECTOR_ACCELEROMETER);
}

bno055_vector_t BNO055::getVectorMagnetometer(){
    return getVector(BNO055_VECTOR_MAGNETOMETER);
}

bno055_vector_t BNO055::getVectorGyroscope(){
    return getVector(BNO055_VECTOR_GYROSCOPE);
}

bno055_vector_t BNO055::getVectorEuler(){
    return getVector(BNO055_VECTOR_EULER);
}

bno055_vector_t BNO055::getVectorLinearAccel(){
    return getVector(BNO055_VECTOR_LINEARACCEL);
}

bno055_vector_t BNO055::getVectorGravity(){
    return getVector(BNO055_VECTOR_GRAVITY);
}

bno055_quaternion_t BNO055::getQuaternion(){
    uint8_t* buffer = (uint8_t*) malloc(8);
    double scale = 1<<14;
    try{
        /* Read quat data (8 bytes) */
        readLen(BNO055_REG_QUA_DATA_W_LSB, 8, buffer);
    }
    catch (std::exception exc){
        free(buffer);
        throw exc;
    }
    bno055_quaternion_t wxyz;
    wxyz.w = ((((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]))/scale;
    wxyz.x = ((((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]))/scale;
    wxyz.y = ((((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]))/scale;
    wxyz.z = ((((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]))/scale;
    free(buffer);
    return wxyz;
}

bno055_offsets_t BNO055::getSensorOffsets(){
    if (_mode != BNO055_OPERATION_MODE_CONFIG){
        throw BNO055WrongOprMode("getSensorOffsets requires BNO055_OPERATION_MODE_CONFIG");
    }
    setPage(0);
    /* Accel offset range depends on the G-range:
        +/-2g  = +/- 2000 mg
        +/-4g  = +/- 4000 mg
        +/-8g  = +/- 8000 mg
        +/-1g = +/- 16000 mg
    */
    uint8_t* buffer = (uint8_t*) malloc(22);
    readLen(BNO055_REG_ACC_OFFSET_X_LSB, 22, buffer);
    bno055_offsets_t sensorOffsets;
    sensorOffsets.accelOffsetX = (buffer[0]) | (buffer[1] << 8);
    sensorOffsets.accelOffsetY = (buffer[2]) | (buffer[3] << 8);
    sensorOffsets.accelOffsetZ = (buffer[4]) | (buffer[5] << 8);

    /* Magnetometer offset range = +/- 6400 LSB where 1uT = 16 LSB */
    sensorOffsets.magOffsetX = (buffer[6]) | (buffer[7] << 8);
    sensorOffsets.magOffsetY = (buffer[8]) | (buffer[9] << 8);
    sensorOffsets.magOffsetZ = (buffer[10]) | (buffer[11] << 8);

    /* Gyro offset range depends on the DPS range:
        2000 dps = +/- 32000 LSB
        1000 dps = +/- 16000 LSB
        500 dps = +/- 8000 LSB
        250 dps = +/- 4000 LSB
        125 dps = +/- 2000 LSB
        ... where 1 DPS = 16 LSB
    */
    sensorOffsets.gyroOffsetX = (buffer[12]) | (buffer[13] << 8);
    sensorOffsets.gyroOffsetY = (buffer[14]) | (buffer[15] << 8);
    sensorOffsets.gyroOffsetZ = (buffer[16]) | (buffer[17] << 8);

    /* Accelerometer radius = +/- 1000 LSB */
    sensorOffsets.accelRadius = (buffer[18]) | (buffer[19] << 8);

    /* Magnetometer radius = +/- 960 LSB */
    sensorOffsets.magRadius = (buffer[20]) | (buffer[21] << 8);

    return sensorOffsets;
}

void BNO055::setSensorOffsets(bno055_offsets_t newOffsets){
    if (_mode != BNO055_OPERATION_MODE_CONFIG){
        throw BNO055WrongOprMode("setSensorOffsets requires BNO055_OPERATION_MODE_CONFIG");
    }
    setPage(0);
    write8(BNO055_REG_ACC_OFFSET_X_LSB, (newOffsets.accelOffsetX & 0xFF));
    write8(BNO055_REG_ACC_OFFSET_X_MSB, ((newOffsets.accelOffsetX >> 8) & 0xFF));

    write8(BNO055_REG_ACC_OFFSET_Y_LSB, (newOffsets.accelOffsetY & 0xFF));
    write8(BNO055_REG_ACC_OFFSET_Y_MSB, ((newOffsets.accelOffsetY >> 8) & 0xFF));

    write8(BNO055_REG_ACC_OFFSET_Z_LSB, (newOffsets.accelOffsetZ & 0xFF));
    write8(BNO055_REG_ACC_OFFSET_Z_MSB, ((newOffsets.accelOffsetZ >> 8) & 0xFF));

    write8(BNO055_REG_MAG_OFFSET_X_LSB, (newOffsets.magOffsetX & 0xFF));
    write8(BNO055_REG_MAG_OFFSET_X_MSB, ((newOffsets.magOffsetX >> 8) & 0xFF));

    write8(BNO055_REG_MAG_OFFSET_Y_LSB, (newOffsets.magOffsetY & 0xFF));
    write8(BNO055_REG_MAG_OFFSET_Y_MSB, ((newOffsets.magOffsetY >> 8) & 0xFF));

    write8(BNO055_REG_MAG_OFFSET_Z_LSB, (newOffsets.magOffsetZ & 0xFF));
    write8(BNO055_REG_MAG_OFFSET_Z_MSB, ((newOffsets.magOffsetZ >> 8) & 0xFF));

    write8(BNO055_REG_GYR_OFFSET_X_LSB, (newOffsets.gyroOffsetX & 0xFF));
    write8(BNO055_REG_GYR_OFFSET_X_MSB, ((newOffsets.gyroOffsetX >> 8) & 0xFF));

    write8(BNO055_REG_GYR_OFFSET_Y_LSB, (newOffsets.gyroOffsetY & 0xFF));
    write8(BNO055_REG_GYR_OFFSET_Y_MSB, ((newOffsets.gyroOffsetY >> 8) & 0xFF));

    write8(BNO055_REG_GYR_OFFSET_Z_LSB, (newOffsets.gyroOffsetZ & 0xFF));
    write8(BNO055_REG_GYR_OFFSET_Z_MSB, ((newOffsets.gyroOffsetZ >> 8) & 0xFF));

    write8(BNO055_REG_ACC_RADIUS_LSB, (newOffsets.accelRadius & 0xFF));
    write8(BNO055_REG_ACC_RADIUS_MSB, ((newOffsets.accelRadius >>8) & 0xFF));

    write8(BNO055_REG_MAG_RADIUS_LSB, (newOffsets.magRadius & 0xFF));
    write8(BNO055_REG_MAG_RADIUS_MSB, ((newOffsets.magRadius >> 8) & 0xFF));
}

void BNO055::enableAccelSlowMotionInterrupt(uint8_t threshold, uint8_t duration, bool xAxis, bool yAxis, bool zAxis, bool useInterruptPin){
    if (_mode != BNO055_OPERATION_MODE_CONFIG){
        throw BNO055WrongOprMode("enableAccelSlowMotionInterrupt requires BNO055_OPERATION_MODE_CONFIG");
    }
    uint8_t tmp = 0;
    setPage(1);
    write8(BNO055_REG_ACC_NM_SET, ((duration << 1) | 0x00)); // duration and Slow Motion flag
    write8(BNO055_REG_ACC_NM_THRES, threshold);
    read8(BNO055_REG_ACC_INT_SETTINGS, &tmp); //read the current value to avoid overwrite of other bits
    write8(BNO055_REG_ACC_INT_SETTINGS, tmp); //update

    //INT_EN
    read8(BNO055_REG_INT_EN, &tmp);
    tmp |= 0x80;

    write8(BNO055_REG_INT_EN, tmp); //update

    //MSK
    if (useInterruptPin == true){
        read8(BNO055_REG_INT_MSK, &tmp);
        tmp |= 0x80;
        write8(BNO055_REG_INT_MSK, tmp); //update
    }
}

void BNO055::enableAccelNoMotionInterrupt(uint8_t threshold, uint8_t duration, bool xAxis, bool yAxis, bool zAxis, bool useInterruptPin){
    if (_mode != BNO055_OPERATION_MODE_CONFIG){
        throw BNO055WrongOprMode("enableAccelNoMotionInterrupt requires BNO055_OPERATION_MODE_CONFIG");
    }
    uint8_t tmp = 0;
    setPage(1);

    write8(BNO055_REG_ACC_NM_SET, ((duration << 1) | 0x01)); // duration and No Motion flag

    write8(BNO055_REG_ACC_NM_THRES, threshold);

    read8(BNO055_REG_ACC_INT_SETTINGS, &tmp); //read the current value to avoid overwrite of other bits

    tmp = (xAxis == true) ? (tmp | 0x04) : (tmp & ~0x04);
    tmp = (yAxis == true) ? (tmp | 0x08) : (tmp & ~0x08);
    tmp = (zAxis == true) ? (tmp | 0x10) : (tmp & ~0x10);

    write8(BNO055_REG_ACC_INT_SETTINGS, tmp); //update
    
    //INT_EN
    read8(BNO055_REG_INT_EN, &tmp);
    tmp |= 0x80;

    write8(BNO055_REG_INT_EN, tmp); //update

    //MSK
    if (useInterruptPin == true){
        read8(BNO055_REG_INT_MSK, &tmp);
        tmp |= 0x80;
        write8(BNO055_REG_INT_MSK, tmp); //update
    }
}

void BNO055::enableAccelAnyMotionInterrupt(uint8_t threshold, uint8_t duration, bool xAxis, bool yAxis, bool zAxis, bool useInterruptPin){
    if (_mode != BNO055_OPERATION_MODE_CONFIG){
        throw BNO055WrongOprMode("enableAccelAnyMotionInterrupt requires BNO055_OPERATION_MODE_CONFIG");
    }
    uint8_t tmp = 0;
    setPage(1);

    write8(BNO055_REG_ACC_AM_THRES, threshold);

    read8(BNO055_REG_ACC_INT_SETTINGS, &tmp); //read the current value to avoid overwrite of other bits

    tmp |= (duration & 0x03);
    tmp = (xAxis == true) ? (tmp | 0x04) : (tmp & ~0x04);
    tmp = (yAxis == true) ? (tmp | 0x08) : (tmp & ~0x08);
    tmp = (zAxis == true) ? (tmp | 0x10) : (tmp & ~0x10);

    write8(BNO055_REG_ACC_INT_SETTINGS, tmp); //update
    //INT_EN
    read8(BNO055_REG_INT_EN, &tmp);
    tmp |= 0x40;

    write8(BNO055_REG_INT_EN, tmp); //update
        
    //MSK
    if (useInterruptPin==true){
        read8(BNO055_REG_INT_MSK, &tmp);
        tmp |= 0x40;
        write8(BNO055_REG_INT_MSK, tmp); //update
    }
}

void BNO055::enableAccelHighGInterrupt(uint8_t threshold, uint8_t duration, bool xAxis, bool yAxis, bool zAxis, bool useInterruptPin){
    if (_mode != BNO055_OPERATION_MODE_CONFIG){
        throw BNO055WrongOprMode("enableAccelHighGInterrupt requires BNO055_OPERATION_MODE_CONFIG");
    }
    uint8_t tmp = 0;
    setPage(1);
    
    write8(BNO055_REG_ACC_HG_THRES, threshold);

    write8(BNO055_REG_ACC_HG_DURATION, duration);

    read8(BNO055_REG_ACC_INT_SETTINGS, &tmp); //read the current value to avoid overwrite of other bits

    tmp = (xAxis == true) ? (tmp | 0x20) : (tmp & ~0x20);
    tmp = (yAxis == true) ? (tmp | 0x40) : (tmp & ~0x40);
    tmp = (zAxis == true) ? (tmp | 0x80) : (tmp & ~0x80);

    write8(BNO055_REG_ACC_INT_SETTINGS, tmp); //update

    //INT_EN
    read8(BNO055_REG_INT_EN, &tmp);
    tmp |= 0x20;
    write8(BNO055_REG_INT_EN, tmp); //update
    
    //MSK
    if (useInterruptPin == true){
        read8(BNO055_REG_INT_MSK, &tmp);
        tmp |= 0x20;
        write8(BNO055_REG_INT_MSK, tmp); //update
    }
}

void BNO055::enableGyroAnyMotionInterrupt(uint8_t threshold, uint8_t slopeSamples, uint8_t awakeDuration, bool xAxis, bool yAxis, bool zAxis, bool filtered, bool useInterruptPin){
    if (_mode != BNO055_OPERATION_MODE_CONFIG){
        throw BNO055WrongOprMode("enableGyroAnyMotionInterrupt requires BNO055_OPERATION_MODE_CONFIG");
    }
    uint8_t tmp = 0;
    setPage(1);
    write8(BNO055_REG_GYR_AM_THRES, threshold);

    tmp |= (awakeDuration & 0x03);
    tmp = (tmp << 2) | (threshold & 0x03);
    write8(BNO055_REG_GYR_AM_SET, tmp);

    read8(BNO055_REG_GYR_INT_SETTING, &tmp); //read the current value to avoid overwrite of other bits
    tmp = (xAxis == true) ? (tmp | 0x01) : (tmp & ~0x01);
    tmp = (yAxis == true) ? (tmp | 0x02) : (tmp & ~0x02);
    tmp = (zAxis == true) ? (tmp | 0x04) : (tmp & ~0x04);
    tmp = (filtered == true ) ? (tmp & ~0x40) : (tmp | 0x40);

    write8(BNO055_REG_GYR_INT_SETTING, tmp); //update
    
    //INT_EN
    read8(BNO055_REG_INT_EN, &tmp);
    tmp |= 0x04;
    write8(BNO055_REG_INT_EN, tmp); //update

    //MSK
    if (useInterruptPin == true){
        read8(BNO055_REG_INT_MSK, &tmp);
        tmp |= 0x04;
        write8(BNO055_REG_INT_MSK, tmp); //update
    }
}

void BNO055::enableGyroHRInterrupt(uint8_t thresholdX, uint8_t durationX, uint8_t hysteresisX,uint8_t thresholdY, uint8_t durationY, uint8_t hysteresisY, uint8_t thresholdZ, uint8_t durationZ, uint8_t hysteresisZ, bool xAxis, bool yAxis, bool zAxis, bool filtered, bool useInterruptPin){
    if (_mode != BNO055_OPERATION_MODE_CONFIG){
        throw BNO055WrongOprMode("enableGyroHRInterrupt requires BNO055_OPERATION_MODE_CONFIG");
    }
    uint8_t tmp = 0;
    setPage(1);

    read8(BNO055_REG_GYR_INT_SETTING, &tmp); //read the current value to avoid overwrite of other bits

    tmp = (xAxis == true) ? (tmp | 0x01) : (tmp & ~0x01);
    tmp = (yAxis == true) ? (tmp | 0x02) : (tmp & ~0x02);
    tmp = (zAxis == true) ? (tmp | 0x04) : (tmp & ~0x04);
    tmp = (filtered == true ) ? (tmp & ~0x40) : (tmp | 0x40);

    write8(BNO055_REG_GYR_INT_SETTING, tmp); //update

    tmp = 0;
    tmp |= (hysteresisX & 0x03);
    tmp = (tmp << 4) | (thresholdX & 0xF);
    write8(BNO055_REG_GYR_HR_X_SET, tmp);

    write8(BNO055_REG_GYR_DUR_X, durationX);

    tmp = 0;
    tmp |= (hysteresisY & 0x03);
    tmp = (tmp << 4) | (thresholdY & 0xF);
    write8(BNO055_REG_GYR_HR_Y_SET, tmp);
    
    write8(BNO055_REG_GYR_DUR_Y, durationY);

    tmp = 0;
    tmp |= (hysteresisZ & 0x03);
    tmp = (tmp << 4) | (thresholdZ & 0xF);
    write8(BNO055_REG_GYR_HR_Z_SET, tmp);
    
    write8(BNO055_REG_GYR_DUR_Z, durationZ);

    //INT_EN
    read8(BNO055_REG_INT_EN, &tmp);
    tmp |= 0x08;
    write8(BNO055_REG_INT_EN, tmp); //update

    //MSK
    if (useInterruptPin == true){
        read8(BNO055_REG_INT_MSK, &tmp);
        tmp |= 0x08;
        write8(BNO055_REG_INT_MSK, tmp); //update
    }
}

void BNO055::setAxisRemap(bno055_axis_config_t config, bno055_axis_sign_t sign){
    if (_mode != BNO055_OPERATION_MODE_CONFIG){
        throw BNO055WrongOprMode("setAxisRemap requires BNO055_OPERATION_MODE_CONFIG");
    }
    setPage(0);
    write8(BNO055_REG_AXIS_MAP_CONFIG, ((uint8_t)config & 0x1F));
    write8(BNO055_REG_AXIS_MAP_SIGN, ((uint8_t)sign & 0x07));
}

void BNO055::setUnits(bno055_accel_unit_t accel, bno055_angular_rate_unit_t angularRate, bno055_euler_unit_t euler, bno055_temperature_unit_t temp, bno055_data_output_format_t format){
    if (_mode != BNO055_OPERATION_MODE_CONFIG){
        throw BNO055WrongOprMode("setUnits requires BNO055_OPERATION_MODE_CONFIG");
    }
    setPage(0);
    uint8_t tmp = 0;
    
    tmp |= accel;
    accelScale = (accel != 0) ? 1 : 100;
    
    tmp |= angularRate;
    angularRateScale = (angularRate != 0) ? 900 : 16;

    tmp |= euler;
    eulerScale = (euler != 0) ? 900 : 16;

    tmp |= temp;
    tempScale = (temp != 0) ? 2 : 1;

    tmp |= format;
    write8(BNO055_REG_UNIT_SEL, tmp);
}

void BNO055::setAccelConfig(bno055_accel_range_t range, bno055_accel_bandwidth_t bandwidth, bno055_accel_opr_mode_t mode){
    if (_mode != BNO055_OPERATION_MODE_CONFIG){
        throw BNO055WrongOprMode("setAccelConfig requires BNO055_OPERATION_MODE_CONFIG");
    }
    setPage(1);
    uint8_t tmp = 0;
    tmp |= range;
    tmp |= bandwidth;
    tmp |= mode;
    write8(BNO055_REG_ACC_CONFIG, tmp);
}

void BNO055::begin(){
    // Setup UART
    esp_err_t esperr = uart_driver_delete(_uartPort);
    uart_param_config(_uartPort, &uart_config);
    uart_set_pin(_uartPort, _txPin, _rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    esperr = uart_driver_install(_uartPort, 128 * 2, 0, 0, NULL, 0);
    if (esperr != ESP_OK){
        throw BNO055UartInitFailed();
    }

    #ifndef BNO055_DEBUG_OFF
    ESP_LOGD(BNO055_LOG_TAG, "Setup UART -> RDY"); //DEBUG
    #endif
    
    if (_intPin != GPIO_NUM_MAX){
        gpio_pad_select_gpio(_intPin);
        gpio_set_direction(_intPin, GPIO_MODE_INPUT);
        gpio_set_intr_type(_intPin, GPIO_INTR_POSEDGE);
        gpio_set_pull_mode(_intPin, GPIO_PULLDOWN_ONLY);
        gpio_intr_enable(_intPin);
        gpio_install_isr_service(0);
        gpio_isr_handler_add(_intPin, bno055_interrupt_handler, (void*) this);
    }
    reset();
    uint8_t id = 0;
    read8(BNO055_REG_CHIP_ID, &id);
    if (id != 0xA0){
        throw BNO055ChipNotDetected(); // this is not the correct device, check your wiring
    }
    setPage(0, true); //forced
    setOpMode(BNO055_OPERATION_MODE_CONFIG, true); // this should be the default OPR_MODE
    write8(BNO055_REG_SYS_TRIGGER, 0x0);
}