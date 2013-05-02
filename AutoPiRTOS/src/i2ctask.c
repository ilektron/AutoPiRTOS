/*****************************************************************************
 *   i2ctest.c:  main C entry file for NXP LPC13xx Family Microprocessors
 *
 *   Copyright(C) 2008, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2008.07.20  ver 1.00    Preliminary version, first Release
 *
******************************************************************************/

/* TODO:
 *	Implement the HMC5883 code
 *	do something with the data collected from the sensors
 */

#include "LPC13xx.h"			/* LPC13xx Peripheral Registers */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "type.h"
// TODO Make an i2ctask.h for all rtos related stuff
#include "i2c.h"
#include "l3g4200d.h"
#include "hmc5883.h"
#include "integration.h"
#include "quaternion.h"

#define i2cGYRO_FREQ		( 200 )
#define i2cMAG_FREQ			( 15 )
#define i2cDELAY			( 1 ) // Go as fast as possible, the task will determine what to do after that
//#define mainI2C_DELAY			( 1000 / (mainI2C_GYRO_FREQ) / (portTICK_RATE_MS) )

#define i2cGYRO_HIGH_PASS_ALPHA		0.999f
#define i2cGYRO_LOW_PASS_ALPHA		0.1f

extern volatile uint32_t I2CCount;
extern volatile uint8_t I2CMasterBuffer[BUFSIZE];
extern volatile uint8_t I2CSlaveBuffer[BUFSIZE];
extern volatile uint32_t I2CMasterState;
extern volatile uint32_t I2CReadLength, I2CWriteLength;

static void prvI2CTask( void *pvParameters );

volatile int16_t gyro_x0 = 0;
volatile int16_t gyro_y0 = 0;
volatile int16_t gyro_z0 = 0;

volatile float gyro_x1 = 0;
volatile float gyro_y1 = 0;
volatile float gyro_z1 = 0;

volatile float gyro_x_pos0 = 0;
volatile float gyro_y_pos0 = 0;
volatile float gyro_z_pos0 = 0;

volatile float gyro_x_pos1 = 0;
volatile float gyro_y_pos1 = 0;
volatile float gyro_z_pos1 = 0;

volatile float gyro_x_pos_filt = 0;
volatile float gyro_y_pos_filt = 0;
volatile float gyro_z_pos_filt = 0;

int createI2CTasks()
{

	xTaskCreate( prvI2CTask, ( signed char * ) "I2C", configMINIMAL_STACK_SIZE, NULL, mainTASK_PRIORITY_I2C, NULL );
	return 0;
}

void zero_angles(void)
{
	gyro_x_pos_filt = 0;
	gyro_y_pos_filt = 0;
	gyro_z_pos_filt = 0;
}

/*******************************************************************************
**   Main Function  main()
*******************************************************************************/
static void prvI2CTask( void *pvParameters )
{
	uint32_t ms = 0;
	uint32_t gyro_count = 0;
	uint32_t mag_count = 0;
	uint8_t gyro_status = 0;
	uint8_t gyro_temp = 0;
	int16_t mag_x, mag_y, mag_z;
	portTickType xNextWakeTime;
	float dt = 1.0f/i2cGYRO_FREQ;

	quaternion attitude;
	quaternion_from_attitude(0,0,0,&attitude);


	float gyro_x0f = 0;
	float gyro_y0f = 0;
	float gyro_z0f = 0;

	// Initialize the hardware
	if ( I2CInit( (uint32_t)I2CMASTER ) == FALSE )	/* initialize I2c */
	{
		while ( 1 );				/* Fatal error */
	}

	gyro_init();
	mag_init();

	// Init the GPIO for LED
//	LPC_GPIO2->DIR |= ( 0x1 << 10 );

	/* Initialize xNextWakeTime - this only needs to be done once. */
	xNextWakeTime = xTaskGetTickCount();


	// Infinite loop to check queue to send and receive data over i2c
	for (;;)
	{
//		ulLEDState = !ulLEDState;
//		LPC_GPIO2->MASKED_ACCESS[ ( 1 << 10) ] = ( ulLEDState << 10 );

		// Check to see if it is time to read from the gyro
		if (ms > (gyro_count * 1000/i2cGYRO_FREQ))
		{


			gyro_status = gyro_get_status();
			gyro_temp = gyro_get_temp();
			gyro_read_rates((int16_t*)&gyro_x0, (int16_t*)&gyro_y0, (int16_t*)&gyro_z0);

			// Convert gyro readings to float
			gyro_x0f = (float)(gyro_x0)/(float)(0x7FFF)*GYRO_SCALE;
			gyro_y0f = (float)(gyro_y0)/(float)(0x7FFF)*GYRO_SCALE;
			gyro_z0f = (float)(gyro_z0)/(float)(0x7FFF)*GYRO_SCALE;

			//Low pass filter gyro raw values
			gyro_x0f = gyro_x1 + i2cGYRO_LOW_PASS_ALPHA * (gyro_x0f - gyro_x1);
			gyro_y0f = gyro_y1 + i2cGYRO_LOW_PASS_ALPHA * (gyro_y0f - gyro_y1);
			gyro_z0f = gyro_z1 + i2cGYRO_LOW_PASS_ALPHA * (gyro_z0f - gyro_z1);


			// Integrate gyro results to form angles
//			gyro_x_pos0 = integrate_trapezoidal(gyro_x0f, gyro_x1, gyro_x_pos0, dt);
//			gyro_x_pos_filt = i2cGYRO_HIGH_PASS_ALPHA*(gyro_x_pos_filt + gyro_x_pos0 - gyro_x_pos1);

//			gyro_y_pos0 = integrate_trapezoidal(gyro_y0f, gyro_y1, gyro_y_pos0, dt);
//			gyro_y_pos_filt = i2cGYRO_HIGH_PASS_ALPHA*(gyro_y_pos_filt + gyro_y_pos0 - gyro_y_pos1);

			// May need to do quaternion math here to eliminate annoying 360degree issue
//			gyro_z_pos0 = integrate_trapezoidal(gyro_z0f, gyro_z1, gyro_z_pos0, dt);
//			gyro_z_pos_filt = i2cGYRO_HIGH_PASS_ALPHA*(gyro_z_pos_filt + gyro_z_pos0 - gyro_z_pos1);

			gyro_count++;

			// Save off the last values
			gyro_x1 = gyro_x0f;
			gyro_y1 = gyro_y0f;
			gyro_z1 = gyro_z0f;

			// Update the attitude estimate
			quaternion_update_with_rates(gyro_x0f, gyro_y0f, gyro_z0f, &attitude, dt);

//			gyro_x_pos1 = gyro_x_pos0;
//			gyro_y_pos1 = gyro_y_pos0;
//			gyro_z_pos1 = gyro_z_pos0;

		}

		// Check to see if we should read the magnetometer
		if (ms > (mag_count * 1000/i2cMAG_FREQ))
		{
			mag_read(&mag_x, &mag_y, &mag_z);
			mag_count++;
		}

		vTaskDelayUntil(&xNextWakeTime, i2cDELAY);
		ms += portTICK_RATE_MS;
	}

	/* In order to start the I2CEngine, the all the parameters
	must be set in advance, including I2CWriteLength, I2CReadLength,
	I2CCmd, and the I2cMasterBuffer which contains the stream
	command/data to the I2c slave device.
	(1) If it's a I2C write only, the number of bytes to be written is
	I2CWriteLength, I2CReadLength is zero, the content will be filled
	in the I2CMasterBuffer.
	(2) If it's a I2C read only, the number of bytes to be read is
	I2CReadLength, I2CWriteLength is 0, the read value will be filled
	in the I2CMasterBuffer.
	(3) If it's a I2C Write/Read with repeated start, specify the
	I2CWriteLength, fill the content of bytes to be written in
	I2CMasterBuffer, specify the I2CReadLength, after the repeated
	start and the device address with RD bit set, the content of the
	reading will be filled in I2CMasterBuffer index at
	I2CMasterBuffer[I2CWriteLength+2].

	e.g. Start, DevAddr(W), WRByte1...WRByteN, Repeated-Start, DevAddr(R),
	RDByte1...RDByteN Stop. The content of the reading will be filled
	after (I2CWriteLength + two devaddr) bytes. */

	/* Write SLA(W), address and one data byte */


	/* Be careful with below fixed delay. From device to device, or
	even same device with different write length, or various I2C clock,
	below delay length may need to be changed accordingly. Having
	a break point before Write/Read start will be helpful to isolate
	the problem. */
//	for ( i = 0; i < 0x200000; i++ );	/* Delay after write */


}

/******************************************************************************
**                            End Of File
******************************************************************************/
