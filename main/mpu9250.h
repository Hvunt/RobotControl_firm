/*
 * MPU9250.h
 *
 *  Created on: 25 ����� 2017 �.
 *      Author: Sergey Popov
 */

#ifndef MPU9250_H_
#define MPU9250_H_

#ifdef __cplusplus
extern "C" {
#endif

//#ifdef STM32F1
//#include "stm32f1xx_hal.h"
//#else
// #include "stm32f0xx_hal.h"
//#endif

// #include "main.h"
#include "driver/i2c.h"
#include <math.h>

/*------===REGISTER MAP===------*/

/*----------ID------------*/
#define MPU9250_ID_ACCELGYR 	0x68		//ID for Accel and Gyro
#define MPU9250_ID_MAGNET 		0x0C
#define MPU9250_ID_ADDR 		0x71

/*-------SELF TEST--------*/
#define MPU9250_SELF_TEST_X 	((uint8_t)0x0D)
#define MPU9250_SELF_TEST_Y 	((uint8_t)0x0E)
#define MPU9250_SELF_TEST_Z 	((uint8_t)0x0F)
#define MPU9250_SELF_TEST_A 	((uint8_t)0x10)

/*---CONFIG AND DIVIDER---*/
#define MPU9250_SMPLRT_DIV 		((uint8_t)0x19)		//only for gyroscope
#define MPU9250_CONFIG		 	((uint8_t)0x1A)
#define MPU9250_GYRO_CONFIG		((uint8_t)0x1B)
#define MPU9250_ACCEL_CONFIG	((uint8_t)0x1C)

/*--ACCELEROMETER CONFIG--*/
#define MPU9250_ACCEL_SCALE_RANGE_2g 	((uint8_t) 0x00)
#define MPU9250_ACCEL_SCALE_RANGE_4g 	((uint8_t) 0x08)
#define MPU9250_ACCEL_SCALE_RANGE_8g 	((uint8_t) 0x10)
#define MPU9250_ACCEL_SCALE_RANGE_16g 	((uint8_t) 0x18)

/*---ACCELEROMETER DATA---*/
#define MPU9250_ACCEL_XOUT_H 	((uint8_t)0x3B)
#define MPU9250_ACCEL_XOUT_L 	((uint8_t)0x3C)
#define MPU9250_ACCEL_YOUT_H 	((uint8_t)0x3D)
#define MPU9250_ACCEL_YOUT_L 	((uint8_t)0x3E)
#define MPU9250_ACCEL_ZOUT_H 	((uint8_t)0x3F)
#define MPU9250_ACCEL_ZOUT_L 	((uint8_t)0x40)

/*----TEMPERATURE DATA----*/
#define MPU9250_TEMP_OUT_H   	((uint8_t)0x41)
#define MPU9250_TEMP_OUT_L	 	((uint8_t)0x42)

/*-----GYROSCOPE DATA-----*/
#define MPU9250_GYRO_XOUT_H  	((uint8_t)0x43)
#define MPU9250_GYRO_XOUT_L  	((uint8_t)0x44)
#define MPU9250_GYRO_YOUT_H  	((uint8_t)0x45)
#define MPU9250_GYRO_YOUT_L  	((uint8_t)0x46)
#define MPU9250_GYRO_ZOUT_H  	((uint8_t)0x47)
#define MPU9250_GYRO_ZOUT_L  	((uint8_t)0x48)

/*---MAGNETOMETER CONFIG--*/
#define MPU9250_MAG_CONFIG_WIA		((uint8_t)0x00)
#define MPU9250_MAG_CONFIG_INFO		((uint8_t)0x01)
#define MPU9250_MAG_CONFIG_ST1		((uint8_t)0x02)
#define MPU9250_MAG_CONFIG_ST2		((uint8_t)0x09)
#define MPU9250_MAG_CONFIG_CNTL		((uint8_t)0x0A)

/*----MAGNETOMETER SAV----*/
#define MPU9250_MAG_ASAX			((uint8_t) 0x10)
#define MPU9250_MAG_ASAY			((uint8_t) 0x11)
#define MPU9250_MAG_ASAZ			((uint8_t) 0x12)

/*----MAGNETOMETER DATA---*/
#define MPU9250_MAG_XOUT_L		((uint8_t) 0x03)
#define MPU9250_MAG_XOUT_H		((uint8_t) 0x04)
#define MPU9250_MAG_YOUT_L		((uint8_t) 0x05)
#define MPU9250_MAG_YOUT_H		((uint8_t) 0x06)
#define MPU9250_MAG_ZOUT_L		((uint8_t) 0x07)
#define MPU9250_MAG_ZOUT_H		((uint8_t) 0x08)

/*-----POWER MANAGMENT----*/
#define MPU9250_PWR_MGMT_1  	((uint8_t)0x6B)
#define MPU9250_PWR_MGMT_2  	((uint8_t)0x6C)

/*------------------------------*/

enum MPU_AccelScales {
	MPU_ACCEL_SCALE_2g,
	MPU_ACCEL_SCALE_4g,
	MPU_ACCEL_SCALE_8g,
	MPU_ACCEL_SCALE_16g
};

uint8_t MPU_ReadData(uint16_t Addr, uint8_t Reg);
void MPU_WriteData(uint16_t Addr, uint8_t Reg, uint8_t Value);

int16_t MPU_GetAccel_X(void);
int16_t MPU_GetAccel_Y(void);
int16_t MPU_GetAccel_Z(void);
void MPU_GetAccel(int16_t *data);
int16_t MPU_GetGyro_X(void);
int16_t MPU_GetGyro_Y(void);
int16_t MPU_GetGyro_Z(void);
void MPU_GetGyro(int16_t *data);
int16_t MPU_GetMag_X(void);
int16_t MPU_GetMag_Y(void);
int16_t MPU_GetMag_Z(void);
void MPU_GetMag(int16_t *data);
int16_t MPU_GetTemp(void);

float MPU_GetYaw(void);
float MPU_GetPitch(void);
float MPU_GetRoll(void);
void MPU_GetAngles(float *data);
float MPU_GetVelocity(void);

void MPU_UpdateAngles(void);

uint8_t MPU_GetAccelSR(void);
void MPU_SetAccelSR(uint8_t scale);
void MPU_WakeUp(void);
void MPU_FallAsleep(void);

uint8_t MPU_ReadAccelGyroID(void);
uint8_t MPU_ReadMagID(void);
void MPU_GetAccelOffset(void);
void MPU_CalibrateMag(void);
void MPU_Init(/*I2C_HandleTypeDef * hi2c*/);
void MPU_Error_handler(void);
//void I2C_ClearBusyFlag(I2C_HandleTypeDef *i2c);

#ifdef __cplusplus
}
#endif
#endif /* MPU9250_H_ */
