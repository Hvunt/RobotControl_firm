#ifndef SLAVE_MCU_H
#define SLAVE_MCU_H

#include "configs.h"
#include "motor_defs.h"
#include <stdint.h>
#include "esp_log.h"
#include "driver/i2c.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

struct slave_mcu
{
    //0 - wheeled, 1 - walked, 2 - hybrid
    uint8_t robot_type;
    //1 - optical, 2 - magnet/rotary, 3 - speed sensor, 0 - error
    uint8_t encoder_type;
    //[0:3] - used terminals M1 = 0, M2, etc. Set bits (M1 = 0s bit, M2 = 1st bit, etc.)
	//[4:8] - position of motor. 1 - left, 0 - right. For example: if fourth bit is 1 then M1 is left motor
    uint8_t dc_ports;
    //1st bit - 1st port, 2nd bit - 2nd port etc.
    uint16_t servo_ports; 
};

typedef struct slave_mcu slave_mcu_t;

void SM_init();
// void SM_sending_task(void *params);
void SM_send_command(uint8_t *data);

#endif