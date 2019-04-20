#ifndef EMAPCONFIG_H_
#define EMAPCONFIG_H_

#include <stdint.h>
#include <string.h>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_spi.h"

#include "bmp280.h"
#include "nRF24L01P.h"

// if error set value and go to end
#define PROCESS_ERROR(x) if (0 != (error = (x))) { goto end; }

//Using modules (1 using / 0 not using)
#define USE_MPU9255				1
#define USE_EXTERNAL_BMP280		0
#define USE_SD					0
#define USE_GPS					0
#define USE_nRF24L01			0

/*****STRUCTURES*****/
typedef enum
{
	EMAP_ERROR_NONE = 0,
	EMAP_ERROR_NO_USE = -10
}EMAP_errors;

typedef struct
{
	int32_t temperature;
	int32_t pressure;
} data_raw_BMP280_t;

typedef struct
{
	float temperature;
	float pressure;
	float height;
} data_BMP280_t;

typedef struct
{
	float accel[3];
	union
	{
		float gyro[3];
		float compass[3];
	};
	float quaternion[4];
} data_MPU9255_t;

typedef struct
{
	uint8_t MPU9255_1;
	uint8_t BMP280_1;
	uint8_t MPU9255_2;
	uint8_t BMP280_2;
	uint8_t SD;
	uint8_t time;
} system_state_t;

typedef struct
{
	float pressure;
	float gyro_staticShift1[3];
	float accel_staticShift1[3];
	float quaternion[4];
	float gyro_staticShift2[3];
	float accel_staticShift2[3];
} system_state_zero_t;


/*****GLOBAL VARIABLES*****/

extern I2C_HandleTypeDef 	i2c_IMU_1;
extern I2C_HandleTypeDef 	i2c_IMU_2;
extern rscs_bmp280_descriptor_t * IMU_bmp280_1;
extern rscs_bmp280_descriptor_t * IMU_bmp280_2;

extern SPI_HandleTypeDef	spi_nRF24L01;
extern nRF24L01P			nRF24;

extern data_raw_BMP280_t 	data_raw_BMP280_1;
extern data_raw_BMP280_t 	data_raw_BMP280_2;
extern data_MPU9255_t 		data_MPU9255_1;
extern data_MPU9255_t 		data_MPU9255_isc;
extern data_MPU9255_t 		data_MPU9255_2;
extern data_BMP280_t 		data_BMP280_1;
extern data_BMP280_t 		data_BMP280_2;
extern system_state_t 		system_state;
extern system_state_zero_t 	system_state_zero;

extern data_MPU9255_t 	data_prev_MPU9255_1;
extern data_MPU9255_t 	data_prev_MPU9255_2;
extern data_MPU9255_t 	data_prev_MPU9255_isc;
extern system_state_t 	system_prev_state;

#endif
