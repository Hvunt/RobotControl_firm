
#include "configs.h"

#include "driver/ledc.h"

#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO       STATUS_LED_IO
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0

enum{
    SL_INIT = 1,
    SL_NORMAL_MODE,
    SL_ERROR = 0xFF
};

void SL_init(void);
void SL_setState(uint8_t _state);
void SL_task(void *params);