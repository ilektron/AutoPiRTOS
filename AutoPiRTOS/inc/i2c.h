/*****************************************************************************
 *   i2c.h:  Header file for NXP LPC13xx Family Microprocessors
 *
 *   Copyright(C) 2006, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2006.07.19  ver 1.00    Preliminary version, first Release
 *
******************************************************************************/
#ifndef __I2C_H 
#define __I2C_H

#define FAST_MODE_PLUS	0

#define BUFSIZE			6
// Timeout after 200ms
#define MAX_TIMEOUT		(200 * portTICK_RATE_MS)

#define I2CMASTER		0x01
#define I2CSLAVE		0x02

#define PCF8594_ADDR	0xA0
#define READ_WRITE		0x01

#define RD_BIT			0x01

#define I2C_IDLE			0
#define I2C_STARTED			1
#define I2C_RESTARTED		2
#define I2C_REPEATED_START	3
#define DATA_ACK			4
#define DATA_NACK			5

#define I2CONSET_I2EN		0x00000040  /* I2C Control Set Register */
#define I2CONSET_AA			0x00000004
#define I2CONSET_SI			0x00000008
#define I2CONSET_STO		0x00000010
#define I2CONSET_STA		0x00000020

#define I2CONCLR_AAC		0x00000004  /* I2C Control clear Register */
#define I2CONCLR_SIC		0x00000008
#define I2CONCLR_STAC		0x00000020
#define I2CONCLR_I2ENC		0x00000040

#define I2DAT_I2C			0x00000000  /* I2C Data Reg */
#define I2ADR_I2C			0x00000000  /* I2C Slave Address Reg */
#define I2SCLH_SCLH			0x00000180  /* I2C SCL Duty Cycle High Reg */
#define I2SCLL_SCLL			0x00000180  /* I2C SCL Duty Cycle Low Reg */
#define I2SCLH_HS_SCLH		0x00000020  /* Fast Plus I2C SCL Duty Cycle High Reg */
#define I2SCLL_HS_SCLL		0x00000020  /* Fast Plus I2C SCL Duty Cycle Low Reg */

#define mainTASK_PRIORITY_I2C		( tskIDLE_PRIORITY + 3 )


extern volatile int16_t gyro_x0, gyro_y0, gyro_z0;
extern volatile float gyro_x_pos_filt, gyro_y_pos_filt, gyro_z_pos_filt;
extern volatile float gyro_x_pos0, gyro_y_pos0, gyro_z_pos0;

int createI2CTasks();

int i2c_write_byte(uint8_t address, uint8_t sub_address, uint8_t byte);
uint8_t i2c_read_byte(uint8_t address, uint8_t sub_addr);
uint8_t i2c_read_bytes(uint8_t address, uint8_t sub_addr, uint32_t num_bytes, uint8_t * bytes);

#define I2C_WRITE_BYTE(ADDRESS, SUBADDR, BYTE)	i2c_write_byte(ADDRESS, SUBADDR, BYTE)
#define I2C_READ_BYTE(ADDRESS, SUB)				i2c_read_byte(ADDRESS, SUB)
#define I2C_READ_BYTES(ADDRESS, SUB, NUM_BYTES)	i2c_read_bytes(ADDRESS, SUB, NUM_BYTES, BYTES)

extern uint32_t I2CInit( uint32_t I2cMode );

#endif /* end __I2C_H */
/****************************************************************************
**                            End Of File
*****************************************************************************/
