

// #include "main.h"
#include "mpu9250.h"
#include "MadgwickAHRS.h"

float EC_getPitch(uint16_t *data);
float EC_getRoll(uint16_t *data);
float EC_getYaw(uint16_t *data);