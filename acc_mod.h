
#ifndef __acc_mod_h
#define __acc_mod_h

#include "stdlib.h"
#include "stdarg.h"
#include "stdint.h"
#include "math.h"
#include "mus_lib.h"

//ADXL345 defines
//(FUTURE IMPLEMENTATION: MPU6050-> http://www.pieter-jan.com/node/11)

#define ACC_ID						0xE5
#define ACC_PWR 					0x2D
#define ACC_FORMAT 				0x31
#define ACC_DATA 					0x32
#define ACC_INT_EN				0x2E
#define ACC_INT_MAP				0x2F
#define ACC_INT_SRC				0x30
#define ACC_TAP_THRESH		0x1D
#define ACC_TAP_DUR				0x21
#define ACC_TAP_LATENT		0x30
#define ACC_TAP_WINDOW		0x30
#define ACC_INACT_THRESH	0x25
#define ACC_INACT_DUR			0x26

//i2c defines
#define I2C1_LOC			0x40005400UL	//base
#define I2C1_ACCESS   ((I2C_REG *) I2C1_LOC)

//function defines
#define accel_init					0
#define accel_ints_init			1
#define accel_read					2
#define accel_read_multi		3
#define accel_write					4
#define accel_write_multi		5

#define PI_CONST						3.14159265358979323846
#define EPSILON							0.0000001

#define CONVERSION_FACTOR		(180/PI_CONST)

typedef struct
{
	int16_t x_data;
	int16_t y_data;
	int16_t z_data;
} acc_data;

typedef struct
{
	int8_t yaw;
	int8_t pitch;
	int8_t roll;
} angle_data;

typedef struct
{
	int8_t xsign 		:		2;
	int8_t ysign 		:		2;
	int8_t zsign 		:		2;
} data_signs;

typedef struct
{
  volatile uint32_t CR1; 
  volatile uint32_t CR2;    
  volatile uint32_t OAR1;  
  volatile uint32_t OAR2;   
  volatile uint32_t DR;     
  volatile uint32_t SR1;    
  volatile uint32_t SR2;      
  volatile uint32_t CCR;  
  volatile uint32_t TRISE;    
  volatile uint32_t FLTR;  
} I2C_REG;

typedef struct
{
	unsigned char is_ints					:		1;	//is the accelerometer responsible for interrupts
	unsigned char is_calibrated		:		1;	//is the accelerometer calibrated yet?
	unsigned char is_upper_link		:		1;	//used for acc closer to shoulder/hips
} acc_meta_data;

//(2.75 words long)
typedef struct
{
	acc_data curr_data;
	angle_data base_angles;
	uint8_t acc_addr;
	acc_meta_data meta_data;
} acc_unit;

//public funcs
uint8_t create_acc(acc_unit*, uint8_t, uint8_t);
uint8_t acc_exec(acc_unit*, uint8_t, ...);

#endif				//__acc_mod_h