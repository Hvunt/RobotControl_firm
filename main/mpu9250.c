/*
 * MPU9250.c
 *
 *  Created on: 25 ����� 2017 �.
 *      Author: Sergey
 */

//==========Headers==========
#include "mpu9250.h"

// #include "driver/i2c.h"

/*static I2C_HandleTypeDef *hi2cP;*/

//===Private constant and other data===

#define sampleFreq	10.0f
//static float const alpha = 0.95;
//static float const rad2deg = 180 / PI;
//static float const deg2rad = PI / 180;
float dT = 0;
//static float const aRes = 2.0f / 32768.0f, gRes = 250.0f / 32768.0f, mRes = 10
//		* 4912.0f / 32760.0f;   // scale resolutions per LSB for the sensors
static float const aRes = 16384.f, gRes = 131, mRes = 0.6f; // scale resolutions per LSB for the sensors
static float magCalFactory[3] = { 0.0f, 0.0f, 0.0f };
static float x_scale_factor = 0.0f, y_scale_factor = 0.0f;
static float x_offset = 0.0f, y_offset = 0.0f;
float velocity_vectors[3];
float accel_offset[3];
//=============================

/**
 * Addr - target device address
 * Reg - internal memory address
 */
uint8_t MPU_ReadData(uint16_t Addr, uint8_t Reg) {
	uint8_t value = 0;
// 	uint8_t status = HAL_I2C_Mem_Read(hi2cP, Addr << 1, (uint16_t) Reg,
// 	I2C_MEMADD_SIZE_8BIT, &value, 1, 10);
// 	if (status == HAL_BUSY) {
// //		I2C_ClearBusyFlag(hi2cP);
// 		value = MPU_ReadData(Addr, Reg);
// 	}
	return value;
}

/**
 * Addr - target device address
 * Reg - internal memory address
 * Value - value for write into device
 */
void MPU_WriteData(uint16_t Addr, uint8_t Reg, uint8_t Value) {
// 	uint8_t status = HAL_I2C_Mem_Write(hi2cP, Addr << 1, (uint16_t) Reg,
// 	I2C_MEMADD_SIZE_8BIT, &Value, 1, 10);
// 	if (status == HAL_BUSY) {
// //		I2C_ClearBusyFlag(hi2cP);
// 		MPU_WriteData(Addr, Reg, Value);
// 	}
}

/*---GET ACCEL DATA---*/
int16_t MPU_GetAccel_X() {
	int16_t RawData = 0;
	RawData = ((MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_ACCEL_XOUT_H) << 8)
			| MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_ACCEL_XOUT_L));
	return RawData;
}

int16_t MPU_GetAccel_Y() {
	int16_t RawData = 0;
	RawData = ((MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_ACCEL_YOUT_H) << 8)
			| MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_ACCEL_YOUT_L));
	return RawData;
}

int16_t MPU_GetAccel_Z() {
	int16_t RawData = 0;
	RawData = ((MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_ACCEL_ZOUT_H) << 8)
			| MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_ACCEL_ZOUT_L));
	return RawData;
}

void MPU_GetAccel(int16_t *data) {
	uint8_t data_address = MPU9250_ACCEL_XOUT_H;
	uint8_t buffer[6] = {0};

	// HAL_I2C_Master_Transmit(hi2cP, MPU9250_ID_ACCELGYR << 1, &data_address, 1,
	// 		100);
	// HAL_I2C_Master_Receive(hi2cP, MPU9250_ID_ACCELGYR << 1, buffer, 6, 100);

	data[0] = ((buffer[0] << 8) | buffer[1]);
	data[1] = ((buffer[2] << 8) | buffer[3]);
	data[2] = ((buffer[4] << 8) | buffer[5]);
}

/*---GET GYRO DATA---*/
int16_t MPU_GetGyro_X() {
	int16_t RawData = 0;
	RawData = ((MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_GYRO_XOUT_H) << 8)
			| MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_GYRO_XOUT_L));
	return RawData;
}

int16_t MPU_GetGyro_Y() {
	int16_t RawData = 0;
	RawData = ((MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_GYRO_YOUT_H) << 8)
			| MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_GYRO_YOUT_L));
	return RawData;
}

int16_t MPU_GetGyro_Z() {
	int16_t RawData = 0;
	RawData = ((MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_GYRO_ZOUT_H) << 8)
			| MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_GYRO_ZOUT_L));
	return RawData;
}

void MPU_GetGyro(int16_t *data) {
	uint8_t data_address = MPU9250_GYRO_XOUT_H;
	uint8_t buffer[6] = {0};

	// HAL_I2C_Master_Transmit(hi2cP, MPU9250_ID_ACCELGYR << 1, &data_address, 1,
	// 		100);
	// HAL_I2C_Master_Receive(hi2cP, MPU9250_ID_ACCELGYR << 1, buffer, 6, 100);

	data[0] = ((buffer[0] << 8) | buffer[1]);
	data[1] = ((buffer[2] << 8) | buffer[3]);
	data[2] = ((buffer[4] << 8) | buffer[5]);
}

/*---GET MAGNET DATA---*/
//this functions does not work
int16_t MPU_GetMag_X() {
	int16_t RawData = 0;
	RawData = ((MPU_ReadData(MPU9250_ID_MAGNET, MPU9250_MAG_XOUT_H) << 8)
			| MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_MAG_XOUT_L));
	return RawData;
}

int16_t MPU_GetMag_Y() {
	int16_t RawData = 0;
	RawData = ((MPU_ReadData(MPU9250_ID_MAGNET, MPU9250_MAG_XOUT_H) << 8)
			| MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_MAG_XOUT_L));
	return RawData;
}

int16_t MPU_GetMag_Z() {
	int16_t RawData = 0;
	RawData = ((MPU_ReadData(MPU9250_ID_MAGNET, MPU9250_MAG_XOUT_H) << 8)
			| MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_MAG_XOUT_L));
	return RawData;
}

//aaand this is work
void MPU_GetMag(int16_t *data) {
	uint8_t buffer[6];

	MPU_WriteData(MPU9250_ID_MAGNET, MPU9250_MAG_CONFIG_CNTL, 0x00);
	// HAL_Delay(2);
	MPU_WriteData(MPU9250_ID_MAGNET, MPU9250_MAG_CONFIG_CNTL, 0x11);
	// HAL_Delay(2);

	uint8_t status = 0;
//	status = MPU_ReadData(MPU9250_ID_ACCELGYR, 0x75);
	while (!status) {
		status = MPU_ReadData(MPU9250_ID_MAGNET, MPU9250_MAG_CONFIG_ST1) & 0x01;
	}
	if (status) {
		// status = HAL_I2C_Mem_Read(hi2cP, MPU9250_ID_MAGNET << 1,
		// MPU9250_MAG_XOUT_L,
		// I2C_MEMADD_SIZE_8BIT, buffer, 6, 100);

		data[0] = ((buffer[1] << 8) | buffer[0]);
		data[1] = ((buffer[3] << 8) | buffer[2]);
		data[2] = ((buffer[5] << 8) | buffer[4]);
	}

	MPU_WriteData(MPU9250_ID_MAGNET, MPU9250_MAG_CONFIG_CNTL, 0x00);
	// HAL_Delay(1);
// 	if (status == HAL_BUSY) {
// //		I2C_ClearBusyFlag(hi2cP);
// 		MPU_GetMag(data);
// 	}
// 	if (status == HAL_ERROR) {
// 		MPU_GetMag(data);
// 	}
}

/*---GET TEMP DATA---*/
int16_t MPU_GetTemp(void) {
	int16_t rawData = 0;
	rawData = MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_TEMP_OUT_H) << 8
			| MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_TEMP_OUT_L);
	rawData = rawData / 340.0f + 36.53f;
	return rawData;
}

/*---GET YAW, PITCH, ROLL, VELOCITY---*/
// float MPU_GetYaw(void) {
// 	return atan2(2.0f * (q[1] * q[2] + q[0] * q[3]),
// 			q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]) * 180 / PI;
// }
// float MPU_GetPitch(void) {
// 	return -asin(2.0f * (q[1] * q[3] - q[0] * q[2])) * 180 / PI;
// }
// float MPU_GetRoll(void) {
// 	return atan2(2.0f * (q[0] * q[1] + q[2] * q[3]),
// 			q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) * 180 / PI;
// }
// void MPU_GetAngles(float *data) {
// 	if (sizeof(data) != 3)
// 		return;
// 	data[0] = MPU_GetYaw();
// 	data[1] = MPU_GetPitch();
// 	data[2] = MPU_GetRoll();
// }

float MPU_GetVelocity(void) {
	int16_t accel_temp[3];
	float accel[3];
	dT = 0.007f;

	MPU_GetAccel(accel_temp);
//	velocity_vectors[2] = 0;
	for(uint8_t i = 0; i < 3; i++){
		accel[i] = accel_temp[i] / aRes - accel_offset[i];
		velocity_vectors[i] = accel[i] * dT;
	}
	return sqrt(velocity_vectors[0] * velocity_vectors[0] + velocity_vectors[1] * velocity_vectors[1] + velocity_vectors[2] * velocity_vectors[2]);
}

/*--UTILITY FUNCTION--*/

uint8_t MPU_GetAccelSR() {
	uint8_t rawData = 0;
	rawData = MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_ACCEL_CONFIG);
	rawData &= MPU9250_ACCEL_SCALE_RANGE_16g;
	return rawData;
}

void MPU_SetAccelSR(uint8_t scale) {
	uint8_t rawData = 0;
	rawData = MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_ACCEL_CONFIG);
	rawData &= ~(MPU9250_ACCEL_SCALE_RANGE_16g); //clearing bits
	switch (scale) {
	case MPU_ACCEL_SCALE_2g:
		rawData |= MPU9250_ACCEL_SCALE_RANGE_2g;
		break;
	case MPU_ACCEL_SCALE_4g:
		rawData |= MPU9250_ACCEL_SCALE_RANGE_4g;
		break;
	case MPU_ACCEL_SCALE_8g:
		rawData |= MPU9250_ACCEL_SCALE_RANGE_8g;
		break;
	case MPU_ACCEL_SCALE_16g:
		rawData |= MPU9250_ACCEL_SCALE_RANGE_16g;
		break;
	default:
		//add checking of status
		break;
	}

	MPU_WriteData(MPU9250_ID_ACCELGYR, MPU9250_ACCEL_CONFIG, rawData);
}

void MPU_WakeUp(void) {
	MPU_WriteData(MPU9250_ID_ACCELGYR, MPU9250_PWR_MGMT_1, 0x00);
}

void MPU_FallAsleep(void) {
	MPU_WriteData(MPU9250_ID_ACCELGYR, MPU9250_PWR_MGMT_1, 0x01);
}

uint8_t MPU_ReadAccelGyroID(void) {
	return MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_ID_ADDR);
}

uint8_t MPU_ReadMagID(void) {
	return MPU_ReadData(MPU9250_ID_MAGNET, MPU9250_MAG_CONFIG_WIA);
}

// void MPU_UpdateAngles(void) {
// 	int16_t accel_raw[3];
// 	int16_t gyro_raw[3];
// 	int16_t mag_raw[3];

// 	MPU_GetAccel(accel_raw);
// 	MPU_GetGyro(gyro_raw);
// 	MPU_GetMag(mag_raw);

// 	float gx = (gyro_raw[0] * PI / 180.0f) / gRes;
// 	float gy = (gyro_raw[1] * PI / 180.0f) / gRes;
// 	float gz = (gyro_raw[2] * PI / 180.0f) / gRes;

// 	float ax = accel_raw[0] / aRes * 9.81f;
// 	float ay = accel_raw[1] / aRes * 9.81f;
// 	float az = accel_raw[2] / aRes * 9.81f;

// 	float mx = mag_raw[0] * x_scale_factor + x_offset;
// 	float my = mag_raw[1] * y_scale_factor + y_offset;
// 	float mz = mag_raw[2];

// 	mx *= mRes;
// 	my *= mRes;
// 	mz *= mRes;

// 	//		MadgwickAHRSupdate(gy, gx, -gz, ay, ax, -az, mx, my, mz, q); // y axis of accel is NORTH
// 	MadgwickAHRSupdate(gx, gy, -gz, ax, ay, -az, my, mx, mz); // x axis of accel is NORTH, y is EAST, -z is DOWN
// 	MPU_GetVelocity();
// }

void MPU_GetAccelOffset(void) {
//	uint8_t data_address;
//	uint8_t buffer[6];
//
//	data_address = 0x77;
//	HAL_I2C_Master_Transmit(hi2cP, MPU9250_ID_ACCELGYR << 1, &data_address, 1,
//			100);
//	HAL_I2C_Master_Receive(hi2cP, MPU9250_ID_ACCELGYR << 1, &buffer[0], 2, 100);
//	data_address = 0x7A;
//	HAL_I2C_Master_Transmit(hi2cP, MPU9250_ID_ACCELGYR << 1, &data_address, 1,
//			100);
//	HAL_I2C_Master_Receive(hi2cP, MPU9250_ID_ACCELGYR << 1, &buffer[2], 2, 100);
//	data_address = 0x7D;
//	HAL_I2C_Master_Transmit(hi2cP, MPU9250_ID_ACCELGYR << 1, &data_address, 1,
//			100);
//	HAL_I2C_Master_Receive(hi2cP, MPU9250_ID_ACCELGYR << 1, &buffer[4], 2, 100);
//
//	accel_offset[0] = ((buffer[0] << 8) | buffer[1]);
//	accel_offset[1] = ((buffer[2] << 8) | buffer[3]);
//	accel_offset[2] = ((buffer[4] << 8) | buffer[5]);
//
//	uint8_t mask_bit[3] = { 0, 0, 0 };
//	for (uint8_t i = 0; i < 3; ++i) {
//		if (accel_offset[i] & 0x01)
//			mask_bit[i] = 0x01;
//	}
//
//	for (uint8_t i = 0; i < 3; ++i) {
//		accel_offset[i] -= 4096;
//	}
//
//	buffer[0] = (accel_offset[0] >> 8) & 0xFF;
//	buffer[1] = accel_offset[0] & 0xFF;
//	buffer[1] = buffer[1] | mask_bit[0];
//	buffer[2] = (accel_offset[1] >> 8) & 0xFF;
//	buffer[3] = accel_offset[1] & 0xFF;
//	buffer[3] = buffer[3] | mask_bit[1];
//	buffer[4] = (accel_offset[2] >> 8) & 0xFF;
//	buffer[5] = accel_offset[2] & 0xFF;
//	buffer[5] = buffer[5] | mask_bit[2];
//
//	uint8_t tx_buff[3];
//	data_address = 0x77;
//	tx_buff[0] = data_address;
//	tx_buff[1] = buffer[0];
//	tx_buff[2] = buffer[1];
//	HAL_I2C_Master_Transmit(hi2cP, MPU9250_ID_ACCELGYR << 1, tx_buff, 3, 100);
//	data_address = 0x7A;
//	tx_buff[0] = data_address;
//	tx_buff[1] = buffer[2];
//	tx_buff[2] = buffer[3];
//	HAL_I2C_Master_Transmit(hi2cP, MPU9250_ID_ACCELGYR << 1, &data_address, 2,
//			100);
//	data_address = 0x7D;
//	tx_buff[0] = data_address;
//	tx_buff[1] = buffer[4];
//	tx_buff[2] = buffer[5];
//	HAL_I2C_Master_Transmit(hi2cP, MPU9250_ID_ACCELGYR << 1, &data_address, 2,
//			100);
//
//	data_address = 0x77;
//	HAL_I2C_Master_Transmit(hi2cP, MPU9250_ID_ACCELGYR << 1, &data_address, 1,
//			100);
//	HAL_I2C_Master_Receive(hi2cP, MPU9250_ID_ACCELGYR << 1, &buffer[0], 2, 100);
//	data_address = 0x7A;
//	HAL_I2C_Master_Transmit(hi2cP, MPU9250_ID_ACCELGYR << 1, &data_address, 1,
//			100);
//	HAL_I2C_Master_Receive(hi2cP, MPU9250_ID_ACCELGYR << 1, &buffer[2], 2, 100);
//	data_address = 0x7D;
//	HAL_I2C_Master_Transmit(hi2cP, MPU9250_ID_ACCELGYR << 1, &data_address, 1,
//			100);
//	HAL_I2C_Master_Receive(hi2cP, MPU9250_ID_ACCELGYR << 1, &buffer[4], 2, 100);
	int16_t accel_temp[3];
	MPU_GetAccel(accel_temp);
	for (uint8_t i = 0; i < 3; i++) {
		accel_offset[i] = accel_temp[i] / aRes;
	}
}

void MPU_CalibrateMag(void) {
	//=======MAGNETOMETER=======
	//getting factory calibration from magnetometer
	uint8_t data_address = MPU9250_MAG_ASAX;
	MPU_WriteData(MPU9250_ID_MAGNET, MPU9250_MAG_CONFIG_CNTL, 0x00);
	// HAL_Delay(1);
	MPU_WriteData(MPU9250_ID_MAGNET, MPU9250_MAG_CONFIG_CNTL, 0x0F);
	// HAL_Delay(1);

	//factory calibration

	uint8_t buffer[3];
	// HAL_I2C_Master_Transmit(hi2cP, MPU9250_ID_MAGNET << 1, &data_address, 1,
	// 		100);
	// HAL_I2C_Master_Receive(hi2cP, MPU9250_ID_MAGNET << 1, buffer, 3, 100);

	magCalFactory[0] = (float) (buffer[0] - 128) / 256.0f + 1.0f;
	magCalFactory[1] = (float) (buffer[1] - 128) / 256.0f + 1.0f;
	magCalFactory[2] = (float) (buffer[2] - 128) / 256.0f + 1.0f;

	MPU_WriteData(MPU9250_ID_MAGNET, MPU9250_MAG_CONFIG_CNTL, 0x00);
	// HAL_Delay(1);

	//offset and scale factor
	uint16_t sample_count = 800;
	int16_t mag_max[3] = { 0, 0, 0 }, mag_min[3] = { 0, 0, 0 }, mag_temp[3] = {
			0, 0, 0 };
//	HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, SET);
//	moveByFrag(MOVE_LEFT, 0);
	for (uint16_t i = 0; i < sample_count; i++) {
		MPU_GetMag(mag_temp);
		for (uint8_t j = 0; j < 3; ++j) {
			//find max and min value
			if (mag_max[j] < mag_temp[j])
				mag_max[j] = mag_temp[j];
			if (mag_min[j] > mag_temp[j])
				mag_min[j] = mag_temp[j];
		}
		// HAL_Delay(2);
	}
//	moveByFrag(MOVE_STOP, 0);

	// 0-X; 1-Y
	x_scale_factor = (mag_max[1] - mag_min[1]) / (mag_max[0] - mag_min[0]);
	y_scale_factor = (mag_max[0] - mag_min[0]) / (mag_max[1] - mag_min[1]);
	if (x_scale_factor < 1)
		x_scale_factor = 1;
	if (y_scale_factor < 1)
		y_scale_factor = 1;

	x_offset = ((mag_max[0] - mag_min[0]) / 2 - mag_max[0]) * x_scale_factor;
	y_offset = ((mag_max[1] - mag_min[1]) / 2 - mag_max[1]) * y_scale_factor;
//	HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, RESET);
}

void MPU_Init(/*I2C_HandleTypeDef * hi2c*/) {
	// hi2cP = hi2c;
	MPU_WakeUp();
	// HAL_Delay(10);
	MPU_WriteData(MPU9250_ID_ACCELGYR, 0x37, 0x02);		//enable magnetometer

	MPU_CalibrateMag();
	MPU_GetAccelOffset();
	// dT = HAL_GetTick() / 1000;
}

void MPU_Error_handler(void) {
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}
