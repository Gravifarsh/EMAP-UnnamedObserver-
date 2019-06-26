/*
 * TSL2561.h
 *
 *  Created on: 1 июн. 2019 г.
 *      Author: developer
 */

#ifndef TSL2561_TSL2561_H_
#define TSL2561_TSL2561_H_

#define CONTROL_REG_ADDR 			 0x00
#define CONTROL_REG_VALUE_POWER_DOWN 0x00
#define CONTROL_REG_VALUE_POWER_UP   0x03

#define TSL2561_COMMAND_BIT          0x80    ///< Must be 1
#define TSL2561_CLEAR_BIT            0x40    ///< Clears any pending interrupt (write 1 to clear)
#define TSL2561_WORD_BIT             0x20    ///< 1 = read/write word (rather than byte)
#define TSL2561_BLOCK_BIT            0x10    ///< 1 = using block read/write

#define TSL2561_REGISTER_CONTROL     		0x00 // Control/power register
#define TSL2561_REGISTER_TIMING             0x01 // Set integration time register
#define TSL2561_REGISTER_THRESHHOLDL_LOW    0x02 // Interrupt low threshold low-byte
#define TSL2561_REGISTER_THRESHHOLDL_HIGH   0x03 // Interrupt low threshold high-byte
#define TSL2561_REGISTER_THRESHHOLDH_LOW    0x04 // Interrupt high threshold low-byte
#define TSL2561_REGISTER_THRESHHOLDH_HIGH   0x05 // Interrupt high threshold high-byte
#define TSL2561_REGISTER_INTERRUPT          0x06 // Interrupt settings
#define TSL2561_REGISTER_CRC                0x08 // Factory use only
#define TSL2561_REGISTER_ID                 0x0A // TSL2561 identification setting
#define TSL2561_REGISTER_CHAN0_LOW          0x0C // Light data channel 0, low byte
#define TSL2561_REGISTER_CHAN0_HIGH         0x0D // Light data channel 0, high byte
#define TSL2561_REGISTER_CHAN1_LOW          0x0E // Light data channel 1, low byte
#define TSL2561_REGISTER_CHAN1_HIGH         0x0F  // Light data channel 1, high byte

#define K1C  0x0043 // 0.130 * 2^RATIO_SCALE
#define B1C  0x0204 // 0.0315 * 2^LUX_SCALE
#define M1C  0x01ad // 0.0262 * 2^LUX_SCALE
#define K2C  0x0085 // 0.260 * 2^RATIO_SCALE
#define B2C  0x0228 // 0.0337 * 2^LUX_SCALE
#define M2C  0x02c1 // 0.0430 * 2^LUX_SCALE
#define K3C  0x00c8 // 0.390 * 2^RATIO_SCALE
#define B3C  0x0253 // 0.0363 * 2^LUX_SCALE
#define M3C  0x0363 // 0.0529 * 2^LUX_SCALE
#define K4C  0x010a // 0.520 * 2^RATIO_SCALE
#define B4C  0x0282 // 0.0392 * 2^LUX_SCALE
#define M4C  0x03df // 0.0605 * 2^LUX_SCALE
#define K5C  0x014d // 0.65 * 2^RATIO_SCALE
#define B5C  0x0177 // 0.0229 * 2^LUX_SCALE
#define M5C  0x01dd // 0.0291 * 2^LUX_SCALE
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

#define LUX_SCALE   14     // scale by 2^14
#define RATIO_SCALE 9      // scale ratio by 2^9
#define CH_SCALE           10     // scale channel values by 2^10
#define CHSCALE_TINT0      0x7517 // 322/11 * 2^CH_SCALE
#define CHSCALE_TINT1      0x0fe7 // 322/81 * 2^CH_SCALE

typedef enum
{
	TSL2561_ADDR_GND	= (0x29 << 1),
	TSL2561_ADDR_FLOAT	= (0x39 << 1),
	TSL2561_ADDR_VDD	= (0x49 << 1)
}tsl2561_adress_t;

typedef enum
{
	TSL2561_GAIN_1X = 0,
	TSL2561_GAIN_16X = 1
}tsl2561_gain_t;

typedef enum
{
	TSL2561_TYPE_T = 0,
	TSL2561_TYPE_CS = 1
}tsl2561_type_t;

typedef enum {
	TSL2561_INT_13MS = 0,
	TSL2561_INT_100MS = 1,
	TSL2561_INT_402MS = 2,
	TSL2561_INT_MANUAL = 3
} tsl2561_int_t;

typedef struct
{
	I2C_HandleTypeDef * hi2c;
	tsl2561_adress_t 	addr;
	tsl2561_gain_t		gain;
	tsl2561_type_t		type;
	tsl2561_int_t		intg;
}tsl2561_t;

HAL_StatusTypeDef tsl2561_readReg(tsl2561_t * htsl, uint8_t regAddr, uint8_t * buffer, uint8_t size);
HAL_StatusTypeDef tsl2561_writeReg(tsl2561_t * htsl, uint8_t regAddr, uint8_t buffer, uint8_t size);
HAL_StatusTypeDef tsl2561_readADC(tsl2561_t * htsl, uint16_t * ch0, uint16_t * ch1);
void tsl2561_calcLux(tsl2561_t * htsl, unsigned int * lux,uint16_t * ch0, uint16_t * ch1);

#endif /* TSL2561_TSL2561_H_ */
