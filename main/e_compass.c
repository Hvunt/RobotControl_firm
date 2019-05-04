
#include "e_compass.h"

#define PI 3.14159265358979323846f

static float const aRes = 16384.f, gRes = 131, mRes = 0.6f; // scale resolutions per LSB for the sensors
static float magCalFactory[3] = { 0.0f, 0.0f, 0.0f };
static float x_scale_factor = 0.0f, y_scale_factor = 0.0f;
static float x_offset = 0.0f, y_offset = 0.0f;
float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };

float EC_getPitch(void)
{
    return -asin(2.0f * (q[1] * q[3] - q[0] * q[2])) * 180 / PI;
}
float EC_getRoll(void)
{
    return atan2(2.0f * (q[0] * q[1] + q[2] * q[3]),
			q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) * 180 / PI;
}
float EC_getYaw(void)
{
    return atan2(2.0f * (q[1] * q[2] + q[0] * q[3]),
			q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]) * 180 / PI;
}

void EC_ecTask(void *data)
{
    /*getting calibrated data etc.*/
    while (1)
    {
        int16_t accel_raw[3];
        int16_t gyro_raw[3];
        int16_t mag_raw[3];

        MPU_GetAccel(accel_raw);
        MPU_GetGyro(gyro_raw);
        MPU_GetMag(mag_raw);

        float gx = (gyro_raw[0] * PI / 180.0f) / gRes;
        float gy = (gyro_raw[1] * PI / 180.0f) / gRes;
        float gz = (gyro_raw[2] * PI / 180.0f) / gRes;

        float ax = accel_raw[0] / aRes * 9.81f;
        float ay = accel_raw[1] / aRes * 9.81f;
        float az = accel_raw[2] / aRes * 9.81f;

        float mx = mag_raw[0] * x_scale_factor + x_offset;
        float my = mag_raw[1] * y_scale_factor + y_offset;
        float mz = mag_raw[2];

        mx *= mRes;
        my *= mRes;
        mz *= mRes;
        
        // x axis of accel is NORTH, y is EAST, -z is DOWN
        MadgwickAHRSupdate(gx, gy, -gz, ax, ay, -az, my, mx, mz); 
        vTaskDelay(10);
    }
}