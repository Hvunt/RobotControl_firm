/*
 * MPU9250.c
 *
 *  Created on: 25 ����� 2017 �.
 *      Author: Sergey
 */

//==========Headers==========
#include "mpu9250.h"

//========Static function definitions=====
static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t address, uint8_t *data_wr, uint8_t size);
static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t address, uint8_t *data_rd, uint8_t size);

//===Private constant and other data===

//static float const alpha = 0.95;
//static float const rad2deg = 180 / PI;
//static float const deg2rad = PI / 180;
//static float const aRes = 2.0f / 32768.0f, gRes = 250.0f / 32768.0f, mRes = 10
//		* 4912.0f / 32760.0f;   // scale resolutions per LSB for the sensors
static float const aRes = 16384.f, gRes = 131, mRes = 0.6f; // scale resolutions per LSB for the sensors
static float magCalFactory[3] = {0.0f, 0.0f, 0.0f};
static float x_scale_factor = 0.0f, y_scale_factor = 0.0f;
static float x_offset = 0.0f, y_offset = 0.0f;
// float velocity_vectors[3];
// float accel_offset[3];
//=============================

static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t address, uint8_t *data_wr, uint8_t size)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (address << 1) | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	return ret;
}

static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t address, uint8_t *data_rd, uint8_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( address << 1 ) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * Addr - target device address
 * Reg - internal memory address
 */
uint8_t MPU_ReadData(uint8_t Addr, uint8_t Reg)
{
	uint8_t value = 0;
	i2c_master_write_slave(I2C_MASTER_NUM, Addr, &Reg, 1);
	i2c_master_read_slave(I2C_MASTER_NUM, Addr, &value, 1);
	return value;
}

/**
 * Addr - target device address
 * Reg - internal memory address
 * Value - value for write into device
 */
void MPU_WriteData(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
	uint8_t data[] = {Reg, Value};
	i2c_master_write_slave(I2C_MASTER_NUM, Addr, data, 2);
}

/*---GET ACCEL DATA---*/
int16_t MPU_GetAccel_X()
{
	int16_t RawData = 0;
	RawData = ((MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_ACCEL_XOUT_H) << 8) | MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_ACCEL_XOUT_L));
	return RawData;
}

int16_t MPU_GetAccel_Y()
{
	int16_t RawData = 0;
	RawData = ((MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_ACCEL_YOUT_H) << 8) | MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_ACCEL_YOUT_L));
	return RawData;
}

int16_t MPU_GetAccel_Z()
{
	int16_t RawData = 0;
	RawData = ((MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_ACCEL_ZOUT_H) << 8) | MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_ACCEL_ZOUT_L));
	return RawData;
}

void MPU_GetAccel(int16_t *data)
{
	uint8_t data_address = MPU9250_ACCEL_XOUT_H;
	uint8_t buffer[6] = {0};

	i2c_master_write_slave(I2C_MASTER_NUM, MPU9250_ID_ACCELGYR, &data_address, 1);
	i2c_master_read_slave(I2C_MASTER_NUM, MPU9250_ID_ACCELGYR, buffer, 6);

	data[0] = ((buffer[0] << 8) | buffer[1]);
	data[1] = ((buffer[2] << 8) | buffer[3]);
	data[2] = ((buffer[4] << 8) | buffer[5]);
}

/*---GET GYRO DATA---*/
int16_t MPU_GetGyro_X()
{
	int16_t RawData = 0;
	RawData = ((MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_GYRO_XOUT_H) << 8) | MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_GYRO_XOUT_L));
	return RawData;
}

int16_t MPU_GetGyro_Y()
{
	int16_t RawData = 0;
	RawData = ((MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_GYRO_YOUT_H) << 8) | MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_GYRO_YOUT_L));
	return RawData;
}

int16_t MPU_GetGyro_Z()
{
	int16_t RawData = 0;
	RawData = ((MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_GYRO_ZOUT_H) << 8) | MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_GYRO_ZOUT_L));
	return RawData;
}

void MPU_GetGyro(int16_t *data)
{
	uint8_t data_address = MPU9250_GYRO_XOUT_H;
	uint8_t buffer[6] = {0};

	i2c_master_write_slave(I2C_MASTER_NUM, MPU9250_ID_ACCELGYR, &data_address, 1);
	i2c_master_read_slave(I2C_MASTER_NUM, MPU9250_ID_ACCELGYR, buffer, 6);

	data[0] = ((buffer[0] << 8) | buffer[1]);
	data[1] = ((buffer[2] << 8) | buffer[3]);
	data[2] = ((buffer[4] << 8) | buffer[5]);
}

/*---GET MAGNET DATA---*/
//this functions does not work
int16_t MPU_GetMag_X()
{
	int16_t RawData = 0;
	RawData = ((MPU_ReadData(MPU9250_ID_MAGNET, MPU9250_MAG_XOUT_H) << 8) | MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_MAG_XOUT_L));
	return RawData;
}

int16_t MPU_GetMag_Y()
{
	int16_t RawData = 0;
	RawData = ((MPU_ReadData(MPU9250_ID_MAGNET, MPU9250_MAG_XOUT_H) << 8) | MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_MAG_XOUT_L));
	return RawData;
}

int16_t MPU_GetMag_Z()
{
	int16_t RawData = 0;
	RawData = ((MPU_ReadData(MPU9250_ID_MAGNET, MPU9250_MAG_XOUT_H) << 8) | MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_MAG_XOUT_L));
	return RawData;
}

//aaand this is work
void MPU_GetMag(int16_t *data)
{
	uint8_t buffer[6];

	MPU_WriteData(MPU9250_ID_MAGNET, MPU9250_MAG_CONFIG_CNTL, 0x00);
	vTaskDelay(2);
	MPU_WriteData(MPU9250_ID_MAGNET, MPU9250_MAG_CONFIG_CNTL, 0x11);
	vTaskDelay(5);

	uint8_t status = 0;
	while (!status)
	{
		status = MPU_ReadData(MPU9250_ID_MAGNET, MPU9250_MAG_CONFIG_ST1) & 0x01;
		ESP_LOGI("MPU9250", "HERE NOW");
	}
	if (status)
	{
		uint8_t reg = MPU9250_MAG_XOUT_L;
		i2c_master_write_slave(I2C_MASTER_NUM, MPU9250_ID_MAGNET, &reg, 1);
		i2c_master_read_slave(I2C_MASTER_NUM, MPU9250_ID_MAGNET, buffer, 6);

		data[0] = ((buffer[1] << 8) | buffer[0]);
		data[1] = ((buffer[3] << 8) | buffer[2]);
		data[2] = ((buffer[5] << 8) | buffer[4]);
	}

	MPU_WriteData(MPU9250_ID_MAGNET, MPU9250_MAG_CONFIG_CNTL, 0x00);
}

/*---GET TEMP DATA---*/
int16_t MPU_GetTemp(void)
{
	int16_t rawData = 0;
	rawData = MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_TEMP_OUT_H) << 8 | MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_TEMP_OUT_L);
	rawData = rawData / 340.0f + 36.53f;
	return rawData;
}

// float MPU_GetVelocity(void)
// {
// 	int16_t accel_temp[3];
// 	float accel[3];
// 	dT = 0.007f;

// 	MPU_GetAccel(accel_temp);
// 	//	velocity_vectors[2] = 0;
// 	for (uint8_t i = 0; i < 3; i++)
// 	{
// 		accel[i] = accel_temp[i] / aRes - accel_offset[i];
// 		velocity_vectors[i] = accel[i] * dT;
// 	}
// 	return sqrt(velocity_vectors[0] * velocity_vectors[0] + velocity_vectors[1] * velocity_vectors[1] + velocity_vectors[2] * velocity_vectors[2]);
// }

/*--UTILITY FUNCTION--*/

uint8_t MPU_GetAccelSR()
{
	uint8_t rawData = 0;
	rawData = MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_ACCEL_CONFIG);
	rawData &= MPU9250_ACCEL_SCALE_RANGE_16g;
	return rawData;
}

void MPU_SetAccelSR(uint8_t scale)
{
	uint8_t rawData = 0;
	rawData = MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_ACCEL_CONFIG);
	rawData &= ~(MPU9250_ACCEL_SCALE_RANGE_16g); //clearing bits
	switch (scale)
	{
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

void MPU_WakeUp(void)
{
	MPU_WriteData(MPU9250_ID_ACCELGYR, MPU9250_PWR_MGMT_1, 0x01);
}

void MPU_FallAsleep(void)
{
	MPU_WriteData(MPU9250_ID_ACCELGYR, MPU9250_PWR_MGMT_1, 0x01);
}

uint8_t MPU_ReadAccelGyroID(void)
{
	return MPU_ReadData(MPU9250_ID_ACCELGYR, MPU9250_ID_ADDR);
}

uint8_t MPU_ReadMagID(void)
{
	return MPU_ReadData(MPU9250_ID_MAGNET, MPU9250_MAG_CONFIG_WIA);
}

void MPU_Reset(void){
	MPU_WriteData(MPU9250_ID_ACCELGYR, MPU9250_PWR_MGMT_1, 0x80);
}

void MPU_GetAccelOffset(float * accel_offset)
{
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
	for (uint8_t i = 0; i < 3; i++)
	{
		accel_offset[i] = accel_temp[i] / aRes;
	}
}

void MPU_CalibrateMag(void)
{
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

	magCalFactory[0] = (float)(buffer[0] - 128) / 256.0f + 1.0f;
	magCalFactory[1] = (float)(buffer[1] - 128) / 256.0f + 1.0f;
	magCalFactory[2] = (float)(buffer[2] - 128) / 256.0f + 1.0f;

	MPU_WriteData(MPU9250_ID_MAGNET, MPU9250_MAG_CONFIG_CNTL, 0x00);
	// HAL_Delay(1);

	//offset and scale factor
	uint16_t sample_count = 800;
	int16_t mag_max[3] = {0, 0, 0}, mag_min[3] = {0, 0, 0}, mag_temp[3] = {0, 0, 0};
	//	HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, SET);
	//	moveByFrag(MOVE_LEFT, 0);
	for (uint16_t i = 0; i < sample_count; i++)
	{
		MPU_GetMag(mag_temp);
		for (uint8_t j = 0; j < 3; ++j)
		{
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

void MPU_Init(void)
{
	MPU_Reset();
	vTaskDelay(10);
	MPU_WakeUp();
	vTaskDelay(10);
	MPU_WriteData(MPU9250_ID_ACCELGYR, 0x37, 0x02); //enable magnetometer
	ESP_LOGI("MPU9250", "%d", MPU_ReadData(MPU9250_ID_ACCELGYR, 0x37));
}

void MPU_Error_handler(void)
{
	//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}
