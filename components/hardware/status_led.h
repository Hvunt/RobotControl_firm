
#include "driver/ledc.h"
#include "configs.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO       STATUS_LED_IO
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0

enum{
    SL_INIT = 1,
    SL_NORMAL_MODE,
    SL_WAIT_FOR_CONNECTION_TO_DEVICE,
    SL_ERROR = 0xFF
};

void SL_init(void);
void SL_setState(uint8_t _state);
void SL_task(void *params);