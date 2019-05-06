

#include "mpu9250.h"
#include "MadgwickAHRS.h"

#include "esp_log.h"

float EC_getPitch(void);
float EC_getRoll(void);
float EC_getYaw(void);

void EC_ecTask(void *data);