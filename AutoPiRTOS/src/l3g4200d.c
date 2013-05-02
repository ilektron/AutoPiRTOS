/*
 * l3g4200d.c
 *
 *  Created on: Jan 7, 2012
 *      Author: Stephen.Farnsworth
 */

/* TODO:
 *	Autorange gyro based on if rates are exceeded
 *	Try fifo mode to ensure constant sample rates
 *
 */

#include "type.h"
#include "i2c.h"
#include "FreeRTOS.h"
#include "task.h"

#include "l3g4200d.h"

volatile int16_t gyro_x_offset = 0;
volatile int16_t gyro_y_offset = 0;
volatile int16_t gyro_z_offset = 0;

void gyros_zero(void)
{
	int i = GYRO_ZERO_COUNT;
	int32_t gyro_x_sum = 0;
	int32_t gyro_y_sum = 0;
	int32_t gyro_z_sum = 0;

	for (; i; i--)
	{
		gyro_x_sum += gyro_x0;
		gyro_y_sum += gyro_y0;
		gyro_z_sum += gyro_z0;
		vTaskDelay(GRYO_ZERO_DELAY);
	}

	gyro_x_offset = gyro_x_sum/GYRO_ZERO_COUNT;
	gyro_y_offset = gyro_y_sum/GYRO_ZERO_COUNT;
	gyro_z_offset = gyro_z_sum/GYRO_ZERO_COUNT;

}


uint8_t gyro_get_status()
{
	return I2C_READ_BYTE(L3G4200D_ADDR, L3G4200D_CTRL_REG1);
}

uint8_t gyro_get_temp()
{
	return I2C_READ_BYTE(L3G4200D_ADDR, L3G4200D_OUT_TEMP);
}

int32_t gyro_init()
{
	// TODO: Consider enabling the high pass filter
	I2C_WRITE_BYTE(L3G4200D_ADDR, L3G4200D_CTRL_REG1, L3G4200D_INIT_COMMAND);
	return 0;
}

int32_t gyro_enable_fifo()
{

	return 0;
}

void gyro_read_rates(int16_t* rateX, int16_t* rateY, int16_t* rateZ)
{
	int x, y, z;

	x = I2C_READ_BYTE(L3G4200D_ADDR, L3G4200D_OUT_X_L);
	x |= I2C_READ_BYTE(L3G4200D_ADDR, L3G4200D_OUT_X_H) << 8;
	y = I2C_READ_BYTE(L3G4200D_ADDR, L3G4200D_OUT_Y_L);
	y |= I2C_READ_BYTE(L3G4200D_ADDR, L3G4200D_OUT_Y_H) << 8;
	z = I2C_READ_BYTE(L3G4200D_ADDR, L3G4200D_OUT_Z_L);
	z |= I2C_READ_BYTE(L3G4200D_ADDR, L3G4200D_OUT_Z_H) << 8;
	*rateX = x - gyro_x_offset;
	*rateY = y - gyro_y_offset;
	*rateZ = z - gyro_z_offset;

}
