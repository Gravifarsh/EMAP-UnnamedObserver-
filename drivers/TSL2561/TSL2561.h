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

typedef enum
{
	TSL2561_ADDR_GND	= (0x29 << 1),
	TSL2561_ADDR_FLOAT	= (0x39 << 1),
	TSL2561_ADDR_VDD	= (0x49 << 1)
}tsl2561_adress_t;

typedef struct
{
	I2C_HandleTypeDef * hi2c;
	tsl2561_adress_t addr;
}tsl2561_t;

HAL_StatusTypeDef tsl2561_readReg(I2C_HandleTypeDef * hi2c, uint8_t regAddr, uint8_t * buffer, uint8_t size);
HAL_StatusTypeDef tsl2561_writeReg(I2C_HandleTypeDef * hi2c, uint8_t regAddr, uint8_t buffer, uint8_t size);

#endif /* TSL2561_TSL2561_H_ */
