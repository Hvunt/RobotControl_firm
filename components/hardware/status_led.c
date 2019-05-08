
#include "status_led.h"

ledc_timer_config_t ledc_timer;
ledc_channel_config_t ledc_channel;

static uint8_t state;

void SL_init(void)
{
    ledc_timer.duty_resolution = LEDC_TIMER_10_BIT;
    ledc_timer.freq_hz = 1000;
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

    ledc_set_fade_with_time(ledc_channel.speed_mode,
                            ledc_channel.channel, 4000, 3000);
    ledc_fade_start(ledc_channel.speed_mode,
                    ledc_channel.channel, LEDC_FADE_NO_WAIT);
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
            ledc_set_fade_with_time(ledc_channel.speed_mode,
                                    ledc_channel.channel, 4000, 3000);
            ledc_fade_start(ledc_channel.speed_mode,
                            ledc_channel.channel, LEDC_FADE_NO_WAIT);
            break;
        case SL_NORMAL_MODE:

            break;
        case SL_ERROR:
        default:
            break;
        }
        state = 0;
        vTaskDelay(10);
    }
}