/*
 * TSL2581.h
 *
 *  Created on: 1 июн. 2019 г.
 *      Author: developer
 */

#include <stm32f4xx_hal.h>

#ifndef TSL2581_TSL2581_H_
#define TSL2581_TSL2581_H_

#define CONTROL_REG_VALUE_POWER_DOWN 0x00
#define CONTROL_REG_VALUE_POWER_UP   0x01
#define CONTROL_REG_VALUE_ADC_UP	 0x02

#define TSL2581_COMMAND_BIT          0x80    ///< Must be 1
#define TSL2581_CLEAR_BIT            0x40    ///< Clears any pending interrupt (write 1 to clear)
#define TSL2581_WORD_BIT             0x20    ///< 1 = read/write word (rather than byte)
#define TSL2581_BLOCK_BIT            0x10    ///< 1 = using block read/write
#define TSL2581_CMD_REP_BYTE		 0x00
#define TSL2581_CMD_INC_BYTE		 0x02

#define TSL2581_REGISTER_CONTROL     		0x00 // Control/power register
#define TSL2581_REGISTER_TIMING             0x01 // Set integration time register
#define TSL2581_REGISTER_INTERRUPT          0x02 // Interrupt settings
#define TSL2581_REGISTER_THRESHHOLDL_LOW    0x03 // Interrupt low threshold low-byte
#define TSL2581_REGISTER_THRESHHOLDL_HIGH   0x04 // Interrupt low threshold high-byte
#define TSL2581_REGISTER_THRESHHOLDH_LOW    0x05 // Interrupt high threshold low-byte
#define TSL2581_REGISTER_THRESHHOLDH_HIGH   0x06 // Interrupt high threshold high-byte
#define TSL2581_REGISTER_ANALOG				0x07 // Analog control register
//#define TSL2581_REGISTER_CRC                0x08 // Factory use only
#define TSL2581_REGISTER_ID                 0x12 // TSL2581 identification setting
#define TSL2581_REGISTER_CHAN0_LOW          0x14 // Light data channel 0, low byte
#define TSL2581_REGISTER_CHAN0_HIGH         0x15 // Light data channel 0, high byte
#define TSL2581_REGISTER_CHAN1_LOW          0x16 // Light data channel 1, low byte
#define TSL2581_REGISTER_CHAN1_HIGH         0x17 // Light data channel 1, high byte
#define TSL2581_REGISTER_TIMERLOW	        0x18 // Manual integration time LOW register
#define TSL2581_REGISTER_TIMERHIGH	        0x19 // Manual integration time HIGH register
#define TSL2581_REGISTER_ID2                0x1E // TSL2581 identification setting

#define K1C  0x009A // 0.30 * 2^RATIO_SCALE
#define B1C  0x2148 // 0.130 * 2^LUX_SCALE
#define M1C  0x3d71 // 0.240 * 2^LUX_SCALE

#define K2C  0x00c3 // 0.260 * 2^RATIO_SCALE
#define B2C  0x2a37 // 0.0337 * 2^LUX_SCALE
#define M2C  0x5b30 // 0.0430 * 2^LUX_SCALE

#define K3C  0x00e6 // 0.390 * 2^RATIO_SCALE
#define B3C  0x18ef // 0.0363 * 2^LUX_SCALE
#define M3C  0x2db9 // 0.0529 * 2^LUX_SCALE

#define K4C  0x0114 // 0.520 * 2^RATIO_SCALE
#define B4C  0x0fdf // 0.0392 * 2^LUX_SCALE
#define M4C  0x199a // 0.0605 * 2^LUX_SCALE

#define K5C  0x0114 // 0.65 * 2^RATIO_SCALE
#define B5C  0x0000 // 0.0229 * 2^LUX_SCALE
#define M5C  0x0000 // 0.0291 * 2^LUX_SCALE

#define K6C  0x019a // 0.80 * 2^RATIO_SCALE
#define B6C  0x0101 // 0.0157 * 2^LUX_SCALE
#define M6C  0x0127 // 0.0180 * 2^LUX_SCALE

#define K7C  0x029a // 1.3 * 2^RATIO_SCALE
#define B7C  0x0037 // 0.00338 * 2^LUX_SCALE
#define M7C  0x002b // 0.00260 * 2^LUX_SCALE

#define K8C  0x029a // 1.3 * 2^RATIO_SCALE
#define B8C  0x0000 // 0.000 * 2^LUX_SCALE
#define M8C  0x0000 // 0.000 * 2^LUX_SCALE

#define K1T  0x0040 // 0.125 * 2^RATIO_SCALE
#define B1T  0x01f2 // 0.0304 * 2^LUX_SCALE
#define M1T  0x01be // 0.0272 * 2^LUX_SCALE

#define K2T  0x0080 // 0.250 * 2^RATIO_SCALE
#define B2T  0x0214 // 0.0325 * 2^LUX_SCALE
#define M2T  0x02d1 // 0.0440 * 2^LUX_SCALE

#define K3T  0x00c0 // 0.375 * 2^RATIO_SCALE
#define B3T  0x023f // 0.0351 * 2^LUX_SCALE
#define M3T  0x037b // 0.0544 * 2^LUX_SCALE

#define K4T  0x0100 // 0.50 * 2^RATIO_SCALE
#define B4T  0x0270 // 0.0381 * 2^LUX_SCALE
#define M4T  0x03fe // 0.0624 * 2^LUX_SCALE

#define K5T  0x0138 // 0.61 * 2^RATIO_SCALE
#define B5T  0x016f // 0.0224 * 2^LUX_SCALE
#define M5T  0x01fc // 0.0310 * 2^LUX_SCALE

#define K6T  0x019a // 0.80 * 2^RATIO_SCALE
#define B6T  0x00d2 // 0.0128 * 2^LUX_SCALE
#define M6T  0x00fb // 0.0153 * 2^LUX_SCALE

#define K7T  0x029a // 1.3 * 2^RATIO_SCALE
#define B7T  0x0018 // 0.00146 * 2^LUX_SCALE
#define M7T  0x0012 // 0.00112 * 2^LUX_SCALE

#define K8T  0x029a // 1.3 * 2^RATIO_SCALE
#define B8T  0x0000 // 0.000 * 2^LUX_SCALE
#define M8T  0x0000 // 0.000 * 2^LUX_SCALE

#define LUX_SCALE   16     // scale by 2^14
#define RATIO_SCALE 9      // scale ratio by 2^9
#define CH_SCALE           16     // scale channel values by 2^10
#define CHSCALE_TINT0      0x7517 // 322/11 * 2^CH_SCALE
#define CHSCALE_TINT1      0x0fe7 // 322/81 * 2^CH_SCALE

#define TSL2581_CH0GAIN128X        107 // 128X gain scalar for Ch0
#define TSL2581_CH1GAIN128X        115 // 128X gain scalar for Ch1

typedef enum
{
	TSL2581_ADDR_GND	= (0x29 << 1),
	TSL2581_ADDR_FLOAT	= (0x39 << 1),
	TSL2581_ADDR_VDD	= (0x49 << 1)
}tsl2581_adress_t;

typedef enum
{
	TSL2581_GAIN_1X = 	0,
	TSL2581_GAIN_8X	= 	1,
	TSL2581_GAIN_16X = 	2,
	TSL2581_GAIN_111X = 3
}tsl2581_gain_t;

typedef enum
{
	TSL2581_TYPE_T = 0,
	TSL2581_TYPE_CS = 1
}tsl2581_type_t;

typedef enum {
	TSL2581_INT_2MS		= 255,
	TSL2581_INT_5MS 	= 254,
	TSL2581_INT_51MS	= 237,
	TSL2581_INT_99MS 	= 219,
	TSL2581_INT_199MS 	= 182,
	TSL2581_INT_399MS 	= 108,
	TSL2581_INT_688MS 	= 1,
	TSL2581_INT_MANUAL 	= 0
} tsl2581_int_t;

typedef struct
{
	I2C_HandleTypeDef * hi2c;
	tsl2581_adress_t 	addr;
	tsl2581_gain_t		gain;
	tsl2581_type_t		type;
	tsl2581_int_t		intg;
}tsl2581_t;

HAL_StatusTypeDef tsl2581_start(tsl2581_t * htsl);

HAL_StatusTypeDef tsl2581_readADC(tsl2581_t * htsl, uint16_t * ch0, uint16_t * ch1);
void tsl2581_calcLux(tsl2581_t * htsl, unsigned int * lux,uint16_t * ch0, uint16_t * ch1);

#endif /* TSL2581_TSL2581_H_ */
