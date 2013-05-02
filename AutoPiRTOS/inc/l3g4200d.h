/*
 * l3g4200d.h
 *
 *  Created on: Jan 7, 2012
 *      Author: Stephen.Farnsworth
 */

#ifndef L3G4200D_H_
#define L3G4200D_H_

#define L3G4200D_INIT_COMMAND	0x6F

#define L3G4200D_WHO_AM_I		0x0F
#define L3G4200D_CTRL_REG1		0x20
#define L3G4200D_CTRL_REG2		0x21
#define L3G4200D_CTRL_REG3		0x22
#define L3G4200D_CTRL_REG4		0x23
#define L3G4200D_CTRL_REG5		0x24
#define L3G4200D_REFERENCE		0x25
#define L3G4200D_OUT_TEMP		0x26
#define L3G4200D_STATUS_REG		0x27
#define L3G4200D_OUT_X_L		0x28
#define L3G4200D_OUT_X_H		0x29
#define L3G4200D_OUT_Y_L		0x2A
#define L3G4200D_OUT_Y_H		0x2B
#define L3G4200D_OUT_Z_L		0x2C
#define L3G4200D_OUT_Z_H		0x2D
#define L3G4200D_FIFO_CTRL_REG	0x2E
#define L3G4200D_FIFO_SRC_REG	0x2F
#define L3G4200D_INT1_CFG		0x30
#define L3G4200D_INT1_SRC		0x31
#define L3G4200D_INT1_TSH_XH	0x32
#define L3G4200D_INT1_TSH_XL	0x33
#define L3G4200D_INT1_TSH_YH	0x34
#define L3G4200D_INT1_TSH_YL	0x35
#define L3G4200D_INT1_TSH_ZH	0x36
#define L3G4200D_INT1_TSH_ZL	0x37
#define L3G4200D_INT1_DURATION	0x38

#define L3G4200D_ADDR	0b11010010

#define GYRO_SCALE	250.0f

#define GYRO_ZERO_COUNT				( 200 )
#define GRYO_ZERO_DELAY				( 5 / portTICK_RATE_MS )

#ifndef I2C_WRITE_BYTE
#error Must define an i2c_send_byte function
#endif

#ifndef I2C_READ_BYTE
#error Must define an i2c_rec_byte function
#endif

extern volatile int16_t gyro_x_offset, gyro_y_offset, gyro_z_offset;

// Functions pertaining to the gyro

int gyro_init();
uint8_t gyro_get_status();
uint8_t gyro_get_temp();
void gyro_read_rates(int16_t* rateX, int16_t* rateY, int16_t* rateZ);
void gyros_zero(void);

#endif /* L3G4200D_H_ */
