#ifndef EMAPCONFIG_H_
#define EMAPCONFIG_H_

#include <stdint.h>
#include <string.h>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_spi.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "diag/Trace.h"

#include "DATA_Helper.h"

#include "TSL2581.h"
#include "bmp280.h"
#include "nRF24L01P.h"
#include "lidar.h"

// if error set value and go to end
#define PROCESS_ERROR(x) if (0 != (error = (x))) { goto end; }

// fill struct with zero
#define FILL_STRUCT_WITH_ZERO(x) memset(&x,	0x00, sizeof(x));

//Using modules (1 using / 0 not using)
#define USE_MPU9255				1
#define USE_EXTERNAL_BMP280		0
#define USE_SD					1
#define USE_GPS					0
#define USE_nRF24L01			0
#define USE_TSL2561				0
#define USE_LIDAR				0

/*****STRUCTURES*****/
typedef enum
{
	EMAP_ERROR_NONE = 0,
	EMAP_ERROR_NO_USE = -10
}EMAP_errors_t;

typedef enum
{
	EMAP_STATE_START,
	EMAP_STATE_READY,
	EMAP_STATE_PAYLOAD,
	EMAP_STATE_FALLING
}EMAP_state_t;

typedef struct
{
	int32_t temperature;
	int32_t pressure;
} data_raw_BMP280_t;

typedef struct
{
	uint32_t time;
	float temperature;
	float pressure;
	float height;
} data_BMP280_t;

typedef struct
{
	uint32_t time;
	float accel[3];
	float compass[3];
	union
	{
	float gyro[3];
	float quaternion[4];
	};
} data_MPU9255_t;

typedef struct
{
	uint32_t time;
	uint8_t GlobalState;
	uint8_t MPU9255_1;
	uint8_t BMP280_1;
	uint8_t MPU9255_2;
	uint8_t BMP280_2;
	uint8_t SD;
	uint8_t	nRF;
	uint8_t GPS;
} system_state_t;

typedef struct
{
	uint32_t time;
	float pressure1;
	float pressure2;
	float gyro_staticShift1[3];
	float accel_staticShift1[3];
	float quaternion[4];
	float gyro_staticShift2[3];
	float accel_staticShift2[3];
} system_state_zero_t;

typedef struct
{
	uint32_t time;
	float coords[3];
} data_GPS_t;

typedef struct
{
	uint32_t time;
	uint16_t ch0;		//Visible + Infrared
	uint16_t ch1;		//Infrared only
	uint16_t lux;
} data_TSL_t;

typedef struct
{
	uint32_t time;
	uint32_t dist;
} data_LIDAR_t;

/*****GLOBAL VARIABLES*****/

extern I2C_HandleTypeDef 	i2c_IMU_1;
extern I2C_HandleTypeDef 	i2c_IMU_2;
extern rscs_bmp280_descriptor_t * IMU_bmp280_1;
extern rscs_bmp280_descriptor_t * IMU_bmp280_2;

extern SPI_HandleTypeDef	spi_nRF24L01;
extern nRF24L01P			nRF24;

extern I2C_HandleTypeDef	i2c_tsl2581;
extern tsl2581_t			tsl2581;

extern UART_HandleTypeDef	uart_lidar;
extern lidar_t				lidar;

extern data_GPS_t			data_GPS;
extern data_raw_BMP280_t 	data_raw_BMP280_1;
extern data_raw_BMP280_t 	data_raw_BMP280_2;
extern data_MPU9255_t 		data_MPU9255_1;
extern data_MPU9255_t 		data_MPU9255_isc;
extern data_MPU9255_t 		data_MPU9255_2;
extern data_BMP280_t 		data_BMP280_1;
extern data_BMP280_t 		data_BMP280_2;
extern data_TSL_t			data_TSL;
extern data_LIDAR_t			data_LIDAR;
extern system_state_t 		system_state;
extern system_state_zero_t 	system_state_zero;

extern data_MPU9255_t 	data_prev_MPU9255_1;
extern data_MPU9255_t 	data_prev_MPU9255_2;
extern data_MPU9255_t 	data_prev_MPU9255_isc;
extern system_state_t 	system_prev_state;

extern SemaphoreHandle_t spi_semphr;
extern SemaphoreHandle_t i2c_semphr;

#define SEM_WAIT_MS (10)

#endif
