/*
 * EMAP_Task_IMU.c
 *
 *  Created on: 2 мар. 2019 г.
 *      Author: developer
 */

#include <math.h>
#include <stdlib.h>
#include "diag/Trace.h"

#include "FreeRTOS.h"
#include "task.h"

#include "EMAPConfig.h"

#include "MPU9255.h"
#include "MadgwickAHRS.h"
#include "quaternion.h"

#define INTIFY(fl) (int)fl, (abs( (int)((fl - (int)fl ) * 1000)))

const rscs_bmp280_calibration_values_t * IMU_bmp280_calibration_values_1;

uint8_t bmp280_init(I2C_HandleTypeDef * hi2c, rscs_bmp280_descriptor_t ** dbmp280)
{
	uint8_t error = 0;

	*dbmp280 = rscs_bmp280_initi2c(hi2c, RSCS_BMP280_I2C_ADDR_HIGH);					//создание дескриптора

	rscs_bmp280_parameters_t IMU_bmp280_parameters;
	IMU_bmp280_parameters.pressure_oversampling = RSCS_BMP280_OVERSAMPLING_X4;			//4		16		измерения на один результат
	IMU_bmp280_parameters.temperature_oversampling = RSCS_BMP280_OVERSAMPLING_X2;		//1		2		измерение на один результат
	IMU_bmp280_parameters.standbytyme = RSCS_BMP280_STANDBYTIME_500US;					//0.5ms	62.5ms	время между 2 измерениями
	IMU_bmp280_parameters.filter = RSCS_BMP280_FILTER_X16;								//x16	x16		фильтр
	error = rscs_bmp280_setup(*dbmp280, &IMU_bmp280_parameters);							//запись параметров

	rscs_bmp280_changemode(*dbmp280, RSCS_BMP280_MODE_NORMAL);							//установка режима NORMAL, постоянные измерения

	IMU_bmp280_calibration_values_1 = rscs_bmp280_get_calibration_values(*dbmp280);

	return error;
}

void IMU_Init()
{
	system_state.MPU9255_1 = mpu9255_init(&i2c_IMU_1, I2C1);
	system_state.MPU9255_2 = mpu9255_init(&i2c_IMU_2, I2C2);

	system_state.BMP280_1 = bmp280_init(&i2c_IMU_1, &IMU_bmp280_1);
	system_state.BMP280_2 = bmp280_init(&i2c_IMU_2, &IMU_bmp280_2);

	trace_printf("MPU1: %d\nMPU2: %d\nBMP1: %d\nBMP2: %d\n",
			system_state.MPU9255_1, system_state.MPU9255_2, system_state.BMP280_1, system_state.BMP280_2);


}

uint8_t getGyroStaticShifts(I2C_HandleTypeDef * hi2c ,float * gyro_staticShift)
{
	uint8_t error = 0;
	uint16_t zero_orientCnt = 200;

	for (int i = 0; i < zero_orientCnt; i++)
	{
		int16_t accelData[3] = { 0 };
		int16_t gyroData[3] = { 0 };
		float gyro[3] = { 0 };

		PROCESS_ERROR(mpu9255_readIMU(hi2c, accelData, gyroData));
		mpu9255_recalcGyro(gyroData, gyro);

		for (int m = 0; m < 3; m++)
			gyro_staticShift[m] += gyro[m];
	}

	for (int m = 0; m < 3; m++)
		gyro_staticShift[m] /= zero_orientCnt;

end:
	return error;
}

uint8_t getAccelStaticShift(I2C_HandleTypeDef * hi2c ,float * gyroStaticShift, float * accelStaticShift)
{
	uint8_t error = 0;
	uint16_t zero_orientCnt = 100;
	float time = 0, time_prev = (float)HAL_GetTick() / 1000;

	for (int i = 0; i < zero_orientCnt; i++)
	{
		int16_t accelData[3] = { 0 };
		int16_t gyroData[3]  = { 0 };
		float accel[3]       = { 0 };
		float accel_ISC[3]   = { 0 };
		float gyro[3]        = { 0 };

		PROCESS_ERROR(mpu9255_readIMU(hi2c, accelData, gyroData));
		mpu9255_recalcGyro(gyroData, gyro);
		mpu9255_recalcAccel(accelData, accel);

		time = (float)HAL_GetTick() / 1000;

		for (int k = 0; k < 3; k++)
			gyro[k] -= gyroStaticShift[k];

		float quaternion[4] = { 0 };
		MadgwickAHRSupdateIMU(quaternion, gyro[0], gyro[1], gyro[2],
										accel[0], accel[1], accel[2],
											time - time_prev, 0.033);
		vect_rotate(accel, quaternion, accel_ISC);

		for (int m = 0; m < 3; m++)
			accelStaticShift[m] += accel_ISC[m];

		time_prev = time;
	}

	for (int m = 0; m < 3; m++)
		accelStaticShift[m] /= zero_orientCnt;

end:
	return error;
}

uint8_t getStaticShifts()
{
	uint8_t error = 0;
	float gyroStaticShifts[3] 	= { 0 };
	float accelStaticShifts[3] 	= { 0 };

taskENTER_CRITICAL();

	PROCESS_ERROR( getGyroStaticShifts(&i2c_IMU_1,gyroStaticShifts) );
	PROCESS_ERROR( getAccelStaticShift(&i2c_IMU_1,gyroStaticShifts, accelStaticShifts) );

	for (int i = 0; i < 3; i++)
	{
		system_state_zero.gyro_staticShift1[i] = gyroStaticShifts[i];
		system_state_zero.accel_staticShift1[i] = accelStaticShifts[i];
	}

	PROCESS_ERROR( getGyroStaticShifts(&i2c_IMU_2,gyroStaticShifts) );
	PROCESS_ERROR( getAccelStaticShift(&i2c_IMU_2,gyroStaticShifts, accelStaticShifts) );

	for (int i = 0; i < 3; i++)
	{
		system_state_zero.gyro_staticShift2[i] = gyroStaticShifts[i];
		system_state_zero.accel_staticShift2[i] = accelStaticShifts[i];
	}

end:
taskEXIT_CRITICAL();
	return error;
}

uint8_t updateIMU(I2C_HandleTypeDef * hi2c)
{
	//////	СОБИРАЕМ ДАННЫЕ IMU	//////////////////////
		//	массивы для хранения
		uint8_t error = 0;
		int16_t accelData[3] = { 0 };
		int16_t gyroData[3] = { 0 };
		int16_t compassData[3] = { 0 };
		float accel[3] = { 0 };
		float gyro[3] = { 0 };
		float compass[3] = { 0 };


taskENTER_CRITICAL();
		//	собираем данные
		PROCESS_ERROR(mpu9255_readIMU(hi2c, accelData, gyroData));
		PROCESS_ERROR(mpu9255_readCompass(hi2c, compassData));
		mpu9255_recalcAccel(accelData, accel);
		mpu9255_recalcGyro(gyroData, gyro);
		mpu9255_recalcCompass(compassData, compass);

		float _time = (float)HAL_GetTick() / 1000;

		if (hi2c->Instance == I2C2)
		{
			data_MPU9255_2.time = _time;
			//	пересчитываем их и записываем в структуры
			for (int k = 0; k < 3; k++)
			{
				data_MPU9255_2.accel[k] = accel[k];
				gyro[k] -= system_state_zero.gyro_staticShift2[k];
				data_MPU9255_2.gyro[k] = gyro[k];
				data_MPU9255_2.compass[k] = compass[k];
			}
		}

		else
		{
			system_state.time = _time; // time for quaternion
			data_MPU9255_1.time = _time;
			//	пересчитываем их и записываем в структуры
			for (int k = 0; k < 3; k++)
			{
				data_MPU9255_1.accel[k] = accel[k];
				gyro[k] -= system_state_zero.gyro_staticShift1[k];
				data_MPU9255_1.gyro[k] = gyro[k];
				data_MPU9255_1.compass[k] = compass[k];
			}
taskEXIT_CRITICAL();

			/////////	ОБНОВЛЯЕМ КВАТЕРНИОН  //////////////////
			float quaternion[4] = { 0 };
taskENTER_CRITICAL();
			float dt = _time - system_prev_state.time;
taskEXIT_CRITICAL();
			MadgwickAHRSupdate(quaternion, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], compass[0], compass[1], compass[2], dt, 1);

			//	копируем кватернион в глобальную структуру
taskENTER_CRITICAL();
			data_MPU9255_isc.quaternion[0] = quaternion[0];
			data_MPU9255_isc.quaternion[1] = quaternion[1];
			data_MPU9255_isc.quaternion[2] = quaternion[2];
			data_MPU9255_isc.quaternion[3] = quaternion[3];
taskEXIT_CRITICAL();

			/////////  ПЕРЕВОДИМ ВЕКТОРЫ в ISC  ////////////////
			float accel_ISC[3] = {0, 0, 0};
			float compass_ISC[3] = {0, 0, 0};
			//	ускорение
			vect_rotate(accel, quaternion, accel_ISC);
			//	вектор магнитного поля
			vect_rotate(compass, quaternion, compass_ISC);

			//	копируем векторы в глобальную структуру
taskENTER_CRITICAL();
			for (int i = 0; i < 3; i++)
				accel_ISC[i] -= system_state_zero.accel_staticShift1[i];

			data_MPU9255_isc.accel[0] = accel_ISC[0];
			data_MPU9255_isc.accel[1] = accel_ISC[1];
			data_MPU9255_isc.accel[2] = accel_ISC[2];
			data_MPU9255_isc.compass[0] = compass_ISC[0];
			data_MPU9255_isc.compass[1] = compass_ISC[1];
			data_MPU9255_isc.compass[2] = compass_ISC[2];
		}

end:
taskEXIT_CRITICAL();
		return error;
}

uint8_t updateBMP280(I2C_HandleTypeDef * hi2c, rscs_bmp280_descriptor_t ** dbmp)
{
	uint8_t error = 0;
	int32_t pressure = 0;
	int32_t temp = 0;
	float pressure_f = 0;
	float temp_f = 0;
	float height = 0;

taskENTER_CRITICAL();
	PROCESS_ERROR( rscs_bmp280_read(*dbmp, &pressure, &temp));
	rscs_bmp280_calculate(IMU_bmp280_calibration_values_1, pressure, temp, &pressure_f, &temp_f);

	float zero_pressure = system_state_zero.pressure;
taskEXIT_CRITICAL();

	height = 18400 * log(zero_pressure / pressure_f);

taskENTER_CRITICAL();
	float _time = (float)HAL_GetTick() / 1000;
	if(hi2c->Instance == I2C1)
	{
		data_raw_BMP280_1.pressure = pressure;
		data_raw_BMP280_1.temperature = temp;

		data_BMP280_1.time = _time;
		data_BMP280_1.pressure = pressure_f;
		data_BMP280_1.temperature = temp_f;
		data_BMP280_1.height = height;
	}
	else
	{
		data_raw_BMP280_2.pressure = pressure;
		data_raw_BMP280_2.temperature = temp;

		data_BMP280_2.time = _time;
		data_BMP280_2.pressure = pressure_f;
		data_BMP280_2.temperature = temp_f;
		data_BMP280_2.height = height;
	}

end:
taskEXIT_CRITICAL();

	return error;
}

void savePrevState()
{
	taskENTER_CRITICAL();
		memcpy(&data_prev_MPU9255_1, 	&data_MPU9255_1,	sizeof(data_MPU9255_1));
		memcpy(&data_prev_MPU9255_2, 	&data_MPU9255_2,	sizeof(data_MPU9255_2));
		memcpy(&data_prev_MPU9255_isc,	&data_MPU9255_isc,	sizeof(data_MPU9255_isc));
		memcpy(&system_prev_state,		&system_state,		sizeof(system_state));
	taskEXIT_CRITICAL();
}

static uint8_t setSysStateZero()
{
	uint8_t error = 0;

taskENTER_CRITICAL();
	PROCESS_ERROR( getStaticShifts() );
	//PROCESS_ERROR( updateAll() ); TODO
	system_state_zero.pressure = data_BMP280_1.pressure;
	for(int i = 0; i < 4; i++)
		system_state_zero.quaternion[i] = data_MPU9255_isc.quaternion[i];
end:
taskEXIT_CRITICAL();
	return error;
}

void IMU_Task()
{
	if( setSysStateZero() == EMAP_ERROR_NONE )
		system_state.GlobalState = EMAP_STATE_READY;

	for(;;)
	{
taskENTER_CRITICAL();
		if(!system_state.MPU9255_1)
			system_state.MPU9255_1 = updateIMU(&i2c_IMU_1);
		else
			system_state.MPU9255_1 = mpu9255_init(&i2c_IMU_1, I2C1);
taskEXIT_CRITICAL();

		writeDataMPU(&i2c_IMU_1);

taskENTER_CRITICAL();
		if(!system_state.MPU9255_2)
			system_state.MPU9255_2 = updateIMU(&i2c_IMU_2);
		else
			system_state.MPU9255_2 = mpu9255_init(&i2c_IMU_2, I2C2);
taskEXIT_CRITICAL();

		writeDataMPU(&i2c_IMU_2);

taskENTER_CRITICAL();
		if(!system_state.BMP280_1)
			system_state.BMP280_1 = updateBMP280(&i2c_IMU_1, &IMU_bmp280_1);
		else
			system_state.BMP280_1 = bmp280_init(&i2c_IMU_1, &IMU_bmp280_1);
taskEXIT_CRITICAL();

		writeDataBMP(&i2c_IMU_1);

taskENTER_CRITICAL();
		if(!system_state.BMP280_2)
			system_state.BMP280_2 = updateBMP280(&i2c_IMU_2, &IMU_bmp280_2);
		else
			system_state.BMP280_2 = bmp280_init(&i2c_IMU_2, &IMU_bmp280_2);
taskEXIT_CRITICAL();

		writeDataBMP(&i2c_IMU_2);

		vTaskDelay(10 / portTICK_RATE_MS);


		//trace_printf("Accel1: %d.%d, %d.%d, %d.%d\n", INTIFY(data_MPU9255_1.accel[0]), INTIFY(data_MPU9255_1.accel[1]), INTIFY(data_MPU9255_1.accel[2]));
		//trace_printf("Gyro1: %d.%d, %d.%d, %d.%d\n", INTIFY(data_MPU9255_1.gyro[0]), INTIFY(data_MPU9255_1.gyro[1]), INTIFY(data_MPU9255_1.gyro[2]));
		//trace_printf("Pressure2: %d.%d\nTemp2: %d.%d\n", INTIFY(data_BMP280_2.pressure), INTIFY(data_BMP280_2.temperature));
		//trace_printf("Pressure1: %d.%d\nTemp1: %d.%d\n", INTIFY(data_BMP280_1.pressure), INTIFY(data_BMP280_1.temperature));
		//trace_printf("Accel2: %d.%d, %d.%d, %d.%d\n", INTIFY(data_MPU9255_2.accel[0]), INTIFY(data_MPU9255_2.accel[1]), INTIFY(data_MPU9255_2.accel[2]));
		//trace_printf("Gyro1: %d.%d, %d.%d, %d.%d\n===========\n", INTIFY(data_MPU9255_1.gyro[0]), INTIFY(data_MPU9255_1.gyro[1]), INTIFY(data_MPU9255_1.gyro[2]));
	}
}
