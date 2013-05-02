/*
 * hmc5883.c
 *
 *  Created on: Jan 8, 2012
 *      Author: ilektron
 */

#include "type.h"
#include "i2c.h"

#include "hmc5883.h"

int32_t mag_init()
{
	// Perhaps delay the required 8.3ms after startup.
	I2C_WRITE_BYTE(HMC5883_ADDR, HMC5883_MODE_REG, 0x00);
	return 0;
}

int32_t mag_read(int16_t* magX, int16_t* magY, int16_t* magZ)
{
	int x, y, z;

	x = I2C_READ_BYTE(HMC5883_ADDR, HMC5883_DOUT_X_LSB);
	x |= I2C_READ_BYTE(HMC5883_ADDR, HMC5883_DOUT_X_MSB) << 8;
	y = I2C_READ_BYTE(HMC5883_ADDR, HMC5883_DOUT_Y_LSB);
	y |= I2C_READ_BYTE(HMC5883_ADDR, HMC5883_DOUT_Y_MSB) << 8;
	z = I2C_READ_BYTE(HMC5883_ADDR, HMC5883_DOUT_Z_LSB);
	z |= I2C_READ_BYTE(HMC5883_ADDR, HMC5883_DOUT_Z_MSB) << 8;
	*magX = x;
	*magY = y;
	*magZ = z;

	return 0;
}
