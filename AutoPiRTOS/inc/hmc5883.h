/*
 * hmc5883.h
 *
 *  Created on: Jan 8, 2012
 *      Author: ilektron
 */

#ifndef HMC5883_H_
#define HMC5883_H_

// Insure that the proper I2C functions are declared
#ifndef I2C_WRITE_BYTE
#error Must define an i2c_send_byte function
#endif

#ifndef I2C_READ_BYTE
#error Must define an i2c_rec_byte function
#endif

#define HMC5883_ADDR		0x3C
#define HMC5883_CFG_REG_A	0x00
#define HMC5883_CFG_REG_B	0x01
#define HMC5883_MODE_REG	0x02
#define HMC5883_DOUT_X_MSB	0x03
#define HMC5883_DOUT_X_LSB	0x04
#define HMC5883_DOUT_Z_MSB	0x05
#define HMC5883_DOUT_Z_LSB	0x06
#define HMC5883_DOUT_Y_MSB	0x07
#define HMC5883_DOUT_Y_LSB	0x08
#define HMC5883_STATUS_REG	0x09
#define HMC5883_ID_REG_A	0x0A
#define HMC5883_ID_REG_B	0x0B
#define HMC5883_ID_REG_C	0x0C

int32_t mag_init();
int32_t mag_read(int16_t* magX, int16_t* magY, int16_t* magZ);

#endif /* HMC5883_H_ */
