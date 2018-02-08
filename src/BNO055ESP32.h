/* BNO055ESP32.h
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

#ifndef _BNO055ESP32_H_
#define _BNO055ESP32_H_

#define BNO055_DEBUG_OFF //uncomment this to DISABLE DEBUG LOGS

#include <string>
#include <cstring> //memset, memcpy
#include <exception>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

#ifndef BNO055_DEBUG_OFF
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#endif
#include "esp_log.h"

#define DEFAULT_UART_TIMEOUT_MS 20 // you can try to decrease/increse this.

/* System Status [SYS_STATUS] (sec: 4.3.58)
	0 = Idle
	1 = System Error
	2 = Initializing Peripherals
	3 = System Iniitalization
	4 = Executing Self-Test
	5 = Sensor fusio algorithm running
	6 = System running without fusion algorithms
*/
typedef enum{
	BNO055_SYSTEM_STATUS_IDLE												= 0x0,
	BNO055_SYSTEM_STATUS_SYSTEM_ERROR										= 0x01,
	BNO055_SYSTEM_STATUS_INITIALIZING_PERIPHERALS							= 0x02,
	BNO055_SYSTEM_STATUS_SYSTEM_INITIALIZATION								= 0x03,
	BNO055_SYSTEM_STATUS_EXECUTING_SELF_TEST								= 0x04,
	BNO055_SYSTEM_STATUS_FUSION_ALGO_RUNNING								= 0x05,
	BNO055_SYSTEM_STATUS_FUSION_ALOG_NOT_RUNNING							= 0x06
} bno055_system_status_t;

/* System Error [SYS_ERR] (sec: 4.3.59)
	0 = No error
	1 = Peripheral initialization error
	2 = System initialization error
	3 = Self test result failed
	4 = Register map value out of range
	5 = Register map address out of range
	6 = Register map write error
	7 = BNO low power mode not available for selected operat ion mode
	8 = Accelerometer power mode not available
	9 = Fusion algorithm configuration error
	A = Sensor configuration error
*/
typedef enum{
	BNO055_SYSTEM_ERROR_NO_ERROR											= 0x00,
	BNO055_SYSTEM_ERROR_PERIPHERAL_INITIALIZATION_ERROR						= 0x01,
	BNO055_SYSTEM_ERROR_SYSTEM_INITIALIZATION_ERROR							= 0x02,
	BNO055_SYSTEM_ERROR_SELF_TEST_FAILED									= 0x03,
	BNO055_SYSTEM_ERROR_REG_MAP_VAL_OUT_OF_RANGE							= 0x04,
	BNO055_SYSTEM_ERROR_REG_MAP_ADDR_OUT_OF_RANGE							= 0x05,
	BNO055_SYSTEM_ERROR_REG_MAP_WRITE_ERROR									= 0x06,
	BNO055_SYSTEM_ERROR_LOW_PWR_MODE_NOT_AVAILABLE_FOR_SELECTED_OPR_MODE 	= 0x07,
	BNO055_SYSTEM_ERROR_ACCEL_PWR_MODE_NOT_AVAILABLE 						= 0x08,
	BNO055_SYSTEM_ERROR_FUSION_ALGO_CONF_ERROR								= 0x09,
	BNO055_SYSTEM_ERROR_SENSOR_CONF_ERROR									= 0x0A
} bno055_system_error_t;

typedef enum{
	BNO055_PWR_MODE_NORMAL							= 0x00,
	BNO055_PWR_MODE_LOWPOWER						= 0x01,
	BNO055_PWR_MODE_SUSPEND							= 0x02
} bno055_powermode_t;

typedef enum{
	BNO055_OPERATION_MODE_CONFIG					= 0x00,
	BNO055_OPERATION_MODE_ACCONLY					= 0x01,
	BNO055_OPERATION_MODE_MAGONLY					= 0x02,
	BNO055_OPERATION_MODE_GYRONLY					= 0x03,
	BNO055_OPERATION_MODE_ACCMAG					= 0x04,
	BNO055_OPERATION_MODE_ACCGYRO					= 0x05,
	BNO055_OPERATION_MODE_MAGGYRO					= 0x06,
	BNO055_OPERATION_MODE_AMG						= 0x07,
	BNO055_OPERATION_MODE_IMU						= 0x08,
	BNO055_OPERATION_MODE_COMPASS					= 0x09,
	BNO055_OPERATION_MODE_M4G						= 0x0A,
	BNO055_OPERATION_MODE_NDOF_FMC_OFF				= 0x0B,
	BNO055_OPERATION_MODE_NDOF						= 0x0C
} bno055_opmode_t;

typedef enum{
	BNO055_REMAP_CONFIG_P0							= 0x21,
	BNO055_REMAP_CONFIG_P1							= 0x24,
	BNO055_REMAP_CONFIG_P2							= 0x24,
	BNO055_REMAP_CONFIG_P3							= 0x21,
	BNO055_REMAP_CONFIG_P4							= 0x24,
	BNO055_REMAP_CONFIG_P5							= 0x21,
	BNO055_REMAP_CONFIG_P6							= 0x21,
	BNO055_REMAP_CONFIG_P7							= 0x24
} bno055_axis_config_t;

typedef enum{
	BNO055_REMAP_SIGN_P0							= 0x04,
	BNO055_REMAP_SIGN_P1							= 0x00,
	BNO055_REMAP_SIGN_P2							= 0x06,
	BNO055_REMAP_SIGN_P3							= 0x02,
	BNO055_REMAP_SIGN_P4							= 0x03,
	BNO055_REMAP_SIGN_P5							= 0x01,
	BNO055_REMAP_SIGN_P6							= 0x07,
	BNO055_REMAP_SIGN_P7							= 0x05
} bno055_axis_sign_t;

typedef struct{
	uint8_t mcuState;
	uint8_t gyrState;
	uint8_t magState;
} bno055_self_test_result_t;

typedef struct{
	double x;
	double y;
	double z;
} bno055_vector_t;

typedef struct{
	double w;
	double x;
	double y;
	double z;
} bno055_quaternion_t;

typedef enum{
	BNO055_UNIT_ACCEL_MS2							= 0x00, //m/s²
	BNO055_UNIT_ACCEL_MG							= 0X01
} bno055_accel_unit_t;

typedef enum{
	BNO055_UNIT_ANGULAR_RATE_DPS					= 0x00,
	BNO055_UNIT_ANGULAR_RATE_RPS					= 0x02
} bno055_angular_rate_unit_t;

typedef enum{
	BNO055_UNIT_EULER_DEGREES						= 0x00,
	BNO055_UNIT_EULER_RADIANS						= 0x04
} bno055_euler_unit_t;

typedef enum{
	BNO055_UNIT_TEMP_C								= 0x00,
	BNO055_UNIT_TEMP_F								= 0x10
} bno055_temperature_unit_t;

typedef enum{
	BNO055_DATA_FORMAT_WINDOWS						= 0x00,
	BNO055_DATA_FORMAT_ANDROID						= 0x80
} bno055_data_output_format_t;

typedef struct{
	int16_t accelOffsetX;
    int16_t accelOffsetY;
    int16_t accelOffsetZ;
    
	int16_t magOffsetX;
    int16_t magOffsetY;
    int16_t magOffsetZ;

    int16_t gyroOffsetX;
    int16_t gyroOffsetY;
    int16_t gyroOffsetZ;

    int16_t accelRadius;
    int16_t magRadius;
} bno055_offsets_t;

typedef struct{
	uint8_t sys;
    uint8_t gyro;
    uint8_t mag;
	uint8_t accel;
} bno055_calibration_t;

class BNO055BaseException : public std::exception{
	protected:
	std::string _msg;
	
	public:
	BNO055BaseException(std::string message = ";-(, there was an error."){_msg = message;};
	virtual const char* what() const throw(){
		return _msg.c_str();
	}
};

class BNO055ReadFail : public BNO055BaseException{
	public:
	BNO055ReadFail(std::string message = "(!*)this is specified in datasheet, but it is not in UART Application note, so I don't have an official description.") : BNO055BaseException(message){};
};

class BNO055WriteFail : public BNO055BaseException{
	public:
	BNO055WriteFail(std::string message = "Check connection, protocol settings and operation mode of the BNO055.") : BNO055BaseException(message){};
};

class BNO055RegmapInvalidAddress : public BNO055BaseException{
	public:
	BNO055RegmapInvalidAddress(std::string message = "Check the register is addressable. For example in Page0, should be from 0x38 to 0x6A.") : BNO055BaseException(message){};
};

class BNO055RegmapWriteDisabled : public BNO055BaseException{
	public:
	BNO055RegmapWriteDisabled(std::string message = "Check the property of register.") : BNO055BaseException(message){};
};

class BNO055WrongStartByte : public BNO055BaseException{
	public:
	BNO055WrongStartByte(std::string message = "Check if the first byte send is 0xAA.") : BNO055BaseException(message){};
};

class BNO055BusOverRunError : public BNO055BaseException{
	public:
	BNO055BusOverRunError(std::string message = "Resend the command") : BNO055BaseException(message){};
};

class BNO055MaxLengthError : public BNO055BaseException{
	public:
	BNO055MaxLengthError(std::string message = "Split the command, so that a single frame has less than 128 Bytes.") : BNO055BaseException(message){};
};

class BNO055MinLengthError : public BNO055BaseException{
	public:
	BNO055MinLengthError(std::string message = "Send a valid frame.") : BNO055BaseException(message){};
};

class BNO055ReceiveCharacterTimeout : public BNO055BaseException{
	public:
	BNO055ReceiveCharacterTimeout(std::string message = "Decrease waiting time between sending of two bytes of one frame.") : BNO055BaseException(message){};
};

class BNO055UnknowError : public BNO055BaseException{
	public:
	BNO055UnknowError(std::string message = ".") : BNO055BaseException(message){};
};

class BNO055UartTimeout : public BNO055BaseException{
	public:
	BNO055UartTimeout(std::string message = "bno055 did not answer in time, if you see this often, try to increase timeoutMS.") : BNO055BaseException(message){};
};

class BNO055UartInitFailed : public BNO055BaseException{
	public:
	BNO055UartInitFailed(std::string message = "ESP32's UART Interface cannot be initialized.") : BNO055BaseException(message){};
};

class BNO055ChipNotDetected: public BNO055BaseException{
	public:
	BNO055ChipNotDetected(std::string message = "Check your wiring.") : BNO055BaseException(message){};
};

class BNO055WrongOprMode: public BNO055BaseException{
	public:
	BNO055WrongOprMode(std::string message = "Check the OperationMode.") : BNO055BaseException(message){};
};

class BNO055{
	public:
	// BNO055 Registers(Table 4-1, Pag 51)
	typedef enum{
		// PAGE 1
		BNO055_REG_ACC_CONFIG					= 0x08,
		BN0055_REG_MAG_CONFIG					= 0x09,
		BNO055_REG_GYR_CONFIG_0					= 0x0A,
		BNO055_REG_GYR_CONFIG_1					= 0x0B,

		BNO055_REG_ACC_SLEEP_CONFIG				= 0x0C,
		BNO055_REG_GYR_SLEEP_CONFIG				= 0x0D,

		BNO055_REG_INT_MSK						= 0x0F,
		BNO055_REG_INT_EN						= 0x10,

		BNO055_REG_ACC_AM_THRES					= 0x11,
		BNO055_REG_ACC_INT_SETTINGS				= 0x12,
		BNO055_REG_ACC_HG_DURATION				= 0x13,
		BNO055_REG_ACC_HG_THRES					= 0x14,
		BNO055_REG_ACC_NM_THRES					= 0x15,
		BNO055_REG_ACC_NM_SET					= 0x16,
		BNO055_REG_GYR_INT_SETTING				= 0x17,
		BNO055_REG_GYR_HR_X_SET					= 0x18,
		BNO055_REG_GYR_DUR_X					= 0x19,
		BNO055_REG_GYR_HR_Y_SET					= 0x1A,
		BNO055_REG_GYR_DUR_Y					= 0x1B,
		BNO055_REG_GYR_HR_Z_SET					= 0x1C,
		BNO055_REG_GYR_DUR_Z					= 0x1D,
		BNO055_REG_GYR_AM_THRES					= 0x1E,
		BNO055_REG_GYR_AM_SET					= 0x1F,
		
		BNO055_REG_PAGE_ID						= 0x07,
		
		// PAGE 0
		BNO055_REG_CHIP_ID						= 0x00,
		BNO055_REG_ACC_ID						= 0x01,
		BNO055_REG_MAG_ID						= 0x02,
		BNO055_REG_GYRO_ID						= 0x03,
		BNO055_REG_SW_REV_ID_LSB				= 0x04,
		BNO055_REG_SW_REV_ID_MSB				= 0x05,
		BNO055_REG_BL_REV_ID					= 0x06,

		BNO055_REG_ACC_DATA_X_LSB				= 0x08,
		BNO055_REG_ACC_DATA_X_MSB				= 0x09,
		BNO055_REG_ACC_DATA_Y_LSB				= 0x0A,
		BNO055_REG_ACC_DATA_Y_MSB				= 0x0B,
		BNO055_REG_ACC_DATA_Z_LSB				= 0x0C,
		BNO055_REG_ACC_DATA_Z_MSB				= 0x0D,

		BNO055_REG_MAG_DATA_X_LSB				= 0x0E,
		BNO055_REG_MAG_DATA_X_MSB				= 0x0F,
		BNO055_REG_MAG_DATA_Y_LSB				= 0x10,
		BNO055_REG_MAG_DATA_Y_MSB				= 0x11,
		BNO055_REG_MAG_DATA_Z_LSB				= 0x12,
		BNO055_REG_MAG_DATA_Z_MSB				= 0x13,

		BNO055_REG_GYR_DATA_X_LSB				= 0x14,
		BNO055_REG_GYR_DATA_X_MSB				= 0x15,
		BNO055_REG_GYR_DATA_Y_LSB				= 0x16,
		BNO055_REG_GYR_DATA_Y_MSB				= 0x17,
		BNO055_REG_GYR_DATA_Z_LSB				= 0x18,
		BNO055_REG_GYR_DATA_Z_MSB				= 0x19,

		BNO055_REG_EUL_HEADING_LSB				= 0x1A,
		BNO055_REG_EUL_HEADING_MSB				= 0x1B,
		BNO055_REG_EUL_ROLL_LSB					= 0x1C,
		BNO055_REG_EUL_ROLL_MSB					= 0x1D,
		BNO055_REG_EUL_PITCH_LSB				= 0x1E,
		BNO055_REG_EUL_PITCH_MSB				= 0x1F,

		BNO055_REG_QUA_DATA_W_LSB				= 0x20,
		BNO055_REG_QUA_DATA_W_MSB				= 0x21,
		BNO055_REG_QUA_DATA_X_LSB				= 0x22,
		BNO055_REG_QUA_DATA_X_MSB				= 0x23,
		BNO055_REG_QUA_DATA_Y_LSB				= 0x24,
		BNO055_REG_QUA_DATA_Y_MSB				= 0x25,
		BNO055_REG_QUA_DATA_Z_LSB				= 0x26,
		BNO055_REG_QUA_DATA_Z_MSB				= 0x27,

		BNO055_REG_LIA_DATA_X_LSB				= 0x28,
		BNO055_REG_LIA_DATA_X_MSB				= 0x29,
		BNO055_REG_LIA_DATA_Y_LSB				= 0x2A,
		BNO055_REG_LIA_DATA_Y_MSB				= 0x2B,
		BNO055_REG_LIA_DATA_Z_LSB				= 0x2C,
		BNO055_REG_LIA_DATA_Z_MSB				= 0x2D,

		BNO055_REG_GRV_DATA_X_LSB				= 0x2E,
		BNO055_REG_GRV_DATA_X_MSB				= 0x2F,
		BNO055_REG_GRV_DATA_Y_LSB				= 0x30,
		BNO055_REG_GRV_DATA_Y_MSB				= 0x31,
		BNO055_REG_GRV_DATA_Z_LSB				= 0x32,
		BNO055_REG_GRV_DATA_Z_MSB				= 0x33,

		BNO055_REG_TEMP							= 0x34,

		BNO055_REG_CALIB_STAT					= 0x35,
		BNO055_REG_ST_RESULT					= 0x36,
		BNO055_REG_INT_STA						= 0x37,

		BNO055_REG_SYS_CLK_STAT					= 0x38,
		BNO055_REG_SYS_STATUS					= 0x39,
		BNO055_REG_SYS_ERR						= 0x3A,

		BNO055_REG_UNIT_SEL						= 0x3B,

		BNO055_REG_OPR_MODE						= 0x3D,
		BNO055_REG_PWR_MODE						= 0x3E,
		BNO055_REG_SYS_TRIGGER					= 0x3F,
		BNO055_REG_TEMP_SOURCE					= 0x40,

		BNO055_REG_AXIS_MAP_CONFIG				= 0x41,
		BNO055_REG_AXIS_MAP_SIGN				= 0x42,

		BNO055_REG_ACC_OFFSET_X_LSB				= 0x55,
		BNO055_REG_ACC_OFFSET_X_MSB				= 0x56, 
		BNO055_REG_ACC_OFFSET_Y_LSB				= 0x57,
		BNO055_REG_ACC_OFFSET_Y_MSB				= 0x58,
		BNO055_REG_ACC_OFFSET_Z_LSB				= 0x59,
		BNO055_REG_ACC_OFFSET_Z_MSB				= 0x5A,

		BNO055_REG_MAG_OFFSET_X_LSB				= 0x5B,
		BNO055_REG_MAG_OFFSET_X_MSB				= 0x5C,
		BNO055_REG_MAG_OFFSET_Y_LSB				= 0x5D,
		BNO055_REG_MAG_OFFSET_Y_MSB				= 0x5E,
		BNO055_REG_MAG_OFFSET_Z_LSB				= 0x5F,
		BNO055_REG_MAG_OFFSET_Z_MSB				= 0x60,

		BNO055_REG_GYR_OFFSET_X_LSB				= 0x61,
		BNO055_REG_GYR_OFFSET_X_MSB				= 0x62,
		BNO055_REG_GYR_OFFSET_Y_LSB				= 0x63,
		BNO055_REG_GYR_OFFSET_Y_MSB				= 0x64,
		BNO055_REG_GYR_OFFSET_Z_LSB				= 0x65,
		BNO055_REG_GYR_OFFSET_Z_MSB				= 0x66,

		BNO055_REG_ACC_RADIUS_LSB				= 0x67,
		BNO055_REG_ACC_RADIUS_MSB				= 0x68,

		BNO055_REG_MAG_RADIUS_LSB				= 0x69,
		BNO055_REG_MAG_RADIUS_MSB				= 0x6A
	} bno055_reg_t;

	BNO055(uart_port_t uartPort, gpio_num_t txPin = GPIO_NUM_17, gpio_num_t rxPin = GPIO_NUM_16, gpio_num_t rstPin = GPIO_NUM_MAX, gpio_num_t intPin = GPIO_NUM_MAX);

	void begin();
	void reset();

	void setOpMode(bno055_opmode_t mode, bool forced = false);
	void setPwrMode(bno055_powermode_t pwrMode);

	void setExtCrystalUse(bool state);
	bno055_calibration_t getCalibration();
	
	int8_t getTemp();

	bno055_vector_t getVectorAccelerometer();
	bno055_vector_t getVectorMagnetometer();
	bno055_vector_t getVectorGyroscope();
	bno055_vector_t getVectorEuler();
	bno055_vector_t getVectorLinearAccel();
	bno055_vector_t getVectorGravity();
	bno055_quaternion_t getQuaternion();

	bno055_offsets_t getSensorOffsets();
	void setSensorOffsets(bno055_offsets_t newOffsets);
	
	bno055_system_status_t getSystemStatus();
	bno055_self_test_result_t getSelfTestResult();
	bno055_system_error_t getSystemError();

	void setPage(uint8_t page, bool forced = false);

	void enableAccelSlowMotionInterrupt(uint8_t threshold, uint8_t duration, bool xAxis=true, bool yAxis=true, bool zAxis=true, bool useInterruptPin=true);
	void enableAccelNoMotionInterrupt(uint8_t threshold, uint8_t duration, bool xAxis=true, bool yAxis=true, bool zAxis=true, bool useInterruptPin=true);
	void enableAccelAnyMotionInterrupt(uint8_t threshold, uint8_t duration, bool xAxis=true, bool yAxis=true, bool zAxis=true, bool useInterruptPin=true);
	void enableAccelHighGInterrupt(uint8_t threshold, uint8_t duration, bool xAxis=true, bool yAxis=true, bool zAxis=true, bool useInterruptPin=true);
	void enableGyroAnyMotionInterrupt(uint8_t threshold, uint8_t slopeSamples, uint8_t awakeDuration, bool xAxis, bool yAxis, bool zAxis, bool filtered, bool useInterruptPin=true);
	void enableGyroHRInterrupt(uint8_t thresholdX, uint8_t durationX, uint8_t hysteresisX,uint8_t thresholdY, uint8_t durationY, uint8_t hysteresisY, uint8_t thresholdZ, uint8_t durationZ, uint8_t hysteresisZ, bool xAxis=true, bool yAxis=true, bool zAxis=true, bool filtered=true, bool useInterruptPin=true);
	std::exception getException(uint8_t errcode);

	void readLen(bno055_reg_t reg, uint8_t len, uint8_t *buffer, uint32_t timoutMS = DEFAULT_UART_TIMEOUT_MS);
	void read8(bno055_reg_t reg, uint8_t *val, uint32_t timoutMS = DEFAULT_UART_TIMEOUT_MS);

	void writeLen(bno055_reg_t reg, uint8_t *data, uint8_t len, uint32_t timoutMS = DEFAULT_UART_TIMEOUT_MS);
	void write8(bno055_reg_t reg, uint8_t val, uint32_t timoutMS = DEFAULT_UART_TIMEOUT_MS);

	void setAxisRemap(bno055_axis_config_t config, bno055_axis_sign_t sign);
	void setUnits(bno055_accel_unit_t accel, bno055_angular_rate_unit_t angularRate, bno055_euler_unit_t euler, bno055_temperature_unit_t temp, bno055_data_output_format_t format);


	protected:
		uint8_t UART_ROUND_NUM = 64;
		
		gpio_num_t _rstPin;
		gpio_num_t _intPin;
		
		uint8_t _page;
		bno055_opmode_t _mode;
		uart_port_t _uartPort;
		
		gpio_num_t _txPin;
		gpio_num_t _rxPin;

		uint16_t accelScale = 100;
		uint16_t tempScale = 1;
		uint16_t angularRateScale = 16;
		uint16_t eulerScale = 16;
		uint16_t magScale = 16;

		typedef enum{
			BNO055_VECTOR_ACCELEROMETER						=	0x08, // Default: m/s²
			BNO055_VECTOR_MAGNETOMETER						=	0x0E, // Default: uT
			BNO055_VECTOR_GYROSCOPE							=	0x14, // Default: rad/s
			BNO055_VECTOR_EULER								=	0x1A, // Default: degrees
			BNO055_VECTOR_LINEARACCEL						=	0x28, // Default: m/s²
			BNO055_VECTOR_GRAVITY							=	0x2E  // Default: m/s² 
		} bno055_vector_type_t;

		bno055_vector_t getVector(bno055_vector_type_t vec);
		
		
		const uart_config_t uart_config = {
			.baud_rate = 115200,
			.data_bits = UART_DATA_8_BITS,
			.parity    = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
			.rx_flow_ctrl_thresh = 0,
			.use_ref_tick = false
    	};
};

#endif