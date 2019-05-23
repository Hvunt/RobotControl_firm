
#include "e_compass.h"

#define EC_TAG "e_compass"

#define PI 3.14159265358979323846f

static float const aRes = 16384.f, gRes = 131, mRes = 0.6f; // scale resolutions per LSB for the sensors
static float magCalFactory[3] = { 0.0f, 0.0f, 0.0f };
static float x_scale_factor = 1.0f, y_scale_factor = 1.0f;
static float x_offset = 0.0f, y_offset = 0.0f;
float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };

static float yaw, pitch, roll;

float EC_getPitch(void)
{
    return pitch;
}
float EC_getRoll(void)
{
    return roll;
}
float EC_getYaw(void)
{
    return yaw;
}

float EC_getAccelX(void)
{
    return MPU_GetAccel_X() / aRes * 9.81f;
}

float EC_getAccelY(void)
{
    return MPU_GetAccel_Y() / aRes * 9.81f;
}

float EC_getAccelZ(void)
{
    return MPU_GetAccel_Z() / aRes * 9.81f;
}

void EC_ecTask(void *data)
{
    /*getting calibrated data etc.*/
    MPU_Init();
    vTaskDelay(10 / portTICK_RATE_MS);
    
    while (1)
    {

        int16_t accel_raw[3];
        int16_t gyro_raw[3];
        int16_t mag_raw[3];

        MPU_GetAccel(accel_raw);
        // ESP_LOGI(EC_TAG, "I AM HERE");
        MPU_GetGyro(gyro_raw);
        // ESP_LOGI(EC_TAG, "AND NOW HERE");
        MPU_GetMag(mag_raw);
        // ESP_LOGI(EC_TAG, "AAAND HERE");
        

        float ax = accel_raw[0] / aRes * 9.81f;
        float ay = accel_raw[1] / aRes * 9.81f;
        float az = accel_raw[2] / aRes * 9.81f;
        
        float gx = (gyro_raw[0] * PI / 180.0f) / gRes;
        float gy = (gyro_raw[1] * PI / 180.0f) / gRes;
        float gz = (gyro_raw[2] * PI / 180.0f) / gRes;

        float mx = mag_raw[0] * x_scale_factor + x_offset;
        float my = mag_raw[1] * y_scale_factor + y_offset;
        float mz = mag_raw[2];

        mx *= mRes;
        my *= mRes;
        mz *= mRes;

        // ESP_LOGI(EC_TAG, "MPU ID %d", MPU_ReadAccelGyroID());
        // ESP_LOGI(EC_TAG, "MPU magnetometer ID %d", MPU_ReadMagID());
        // ESP_LOGI(EC_TAG, "Accel x y z %f %f %f", ax, ay, az);
        // ESP_LOGI(EC_TAG, "Gyro x y z %f %f %f", gx, gy, gz );
        // ESP_LOGI(EC_TAG, "Mag x y z %f %f %f", mx, my, mz);
        // ESP_LOGI(EC_TAG, "Temp t %d", MPU_GetTemp());

        // x axis of accel is NORTH, -y is EAST, -z is DOWN
        MadgwickAHRSupdate(gx, gy, -gz, ax, -ay, -az, my, -mx, mz);

        yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), 
                            q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]) * 180 / PI;
        roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]),
			                q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) * 180 / PI;
        pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2])) * 180 / PI;



        // // x axis of accel is NORTH, y is EAST, -z is DOWN
        // MadgwickAHRSupdate(gx, gy, -gz, ax, ay, -az, my, mx, mz); 
        vTaskDelay(1 / portTICK_RATE_MS);
    }
}