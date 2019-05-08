
#include "status_led.h"

static const char *TAG_SL = "SL";

ledc_timer_config_t ledc_timer;
ledc_channel_config_t ledc_channel;

static uint8_t state;

void SL_init(void)
{
    ledc_timer.duty_resolution = LEDC_TIMER_10_BIT;
    ledc_timer.freq_hz = 100;
    ledc_timer.speed_mode = LEDC_HS_MODE;
    ledc_timer.timer_num = LEDC_HS_TIMER;
    ledc_timer_config(&ledc_timer);

    ledc_channel.channel = LEDC_HS_CH0_CHANNEL;
    ledc_channel.duty = 0;
    ledc_channel.gpio_num = LEDC_HS_CH0_GPIO;
    ledc_channel.speed_mode = LEDC_HS_MODE;
    ledc_channel.timer_sel = LEDC_HS_TIMER;
    ledc_channel_config(&ledc_channel);

    ledc_fade_func_install(0);
}

void SL_setState(uint8_t _state)
{
    state = _state;
}

void SL_task(void *params)
{
    while (1)
    {
        switch (state)
        {
        case SL_INIT:
            ledc_set_fade_step_and_start(ledc_channel.speed_mode,
                                         ledc_channel.channel, 1, 512, 100, LEDC_FADE_NO_WAIT);
            ledc_set_fade_step_and_start(ledc_channel.speed_mode,
                                         ledc_channel.channel, 1023, 512, 100, LEDC_FADE_NO_WAIT);
            vTaskDelay(1);
            break;
        case SL_WAIT_FOR_CONNECTION_TO_DEVICE:
            ledc_set_fade_step_and_start(ledc_channel.speed_mode,
                                         ledc_channel.channel, 1, 512, 100, LEDC_FADE_NO_WAIT);
            ledc_set_fade_step_and_start(ledc_channel.speed_mode,
                                         ledc_channel.channel, 1023, 512, 25, LEDC_FADE_NO_WAIT);
            break;
        case SL_NORMAL_MODE:
            ledc_set_fade_time_and_start(ledc_channel.speed_mode,
                                         ledc_channel.channel, 1023, 2000, LEDC_FADE_NO_WAIT);
            ledc_set_fade_time_and_start(ledc_channel.speed_mode,
                                         ledc_channel.channel, 1, 2000, LEDC_FADE_NO_WAIT);
            vTaskDelay(1);
            break;
        case SL_ERROR:
        default:
            break;
        }
        vTaskDelay(1);
    }
}