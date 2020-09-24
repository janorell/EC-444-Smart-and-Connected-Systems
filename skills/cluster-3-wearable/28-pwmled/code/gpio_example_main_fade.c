#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
//meant to fade

#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE

#define LEDC_HS_CH0_GPIO       (4)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0


#define LEDC_TEST_DUTY         (4000)
#define LEDC_TEST_FADE_TIME    (3000)

void app_main(void)
{
    int var;
ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_HS_MODE,
    .timer_num = LEDC_HS_TIMER,
    .duty_resolution = LEDC_TIMER_13_BIT,
    .freq_hz = 5000
};
ledc_timer_config(&ledc_timer);
    
ledc_channel_config_t ledc_channel[10] = {
    {
    .channel    = LEDC_HS_CH0_CHANNEL,
    .gpio_num   = LEDC_HS_CH0_GPIO,
    .speed_mode = LEDC_HS_MODE,
    .timer_sel  = LEDC_HS_TIMER,
    .duty       = 0
    }
};
ledc_channel_config(&ledc_channel[0]);

  ledc_fade_func_install(0);
    while(1)
    {
        printf("Fading up\n");
        for (var = 0; var < 10; var++) {
            ledc_set_fade_with_time(ledc_channel[var].speed_mode, ledc_channel[var].channel, LEDC_TEST_DUTY, LEDC_TEST_FADE_TIME);
            ledc_fade_start(ledc_channel[var].speed_mode, ledc_channel[var].channel, LEDC_FADE_NO_WAIT);
        }
        
        vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);
        
        printf("Fading Down\n");
        
        for (var = 0; var < 10; var++) {
                   ledc_set_fade_with_time(ledc_channel[var].speed_mode, ledc_channel[var].channel, 0, LEDC_TEST_FADE_TIME);
                   ledc_fade_start(ledc_channel[var].speed_mode, ledc_channel[var].channel, LEDC_FADE_NO_WAIT);
               }
        
               vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);
               //vTaskDelay(250 / portTICK_PERIOD_MS);
           }
    }
