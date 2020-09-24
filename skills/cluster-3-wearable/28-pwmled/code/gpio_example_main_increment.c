
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/uart.h"


#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO       (4)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_HS_CH1_GPIO       (19)
#define LEDC_HS_CH1_CHANNEL    LEDC_CHANNEL_1
#define LEDC_TEST_CH_NUM       (4)
#define LEDC_TEST_DUTY0        (0)
#define LEDC_TEST_DUTY1        (450)
#define LEDC_TEST_DUTY2        (900)
#define LEDC_TEST_DUTY3        (1350)
#define LEDC_TEST_DUTY4        (1800)
#define LEDC_TEST_DUTY5        (2250)
#define LEDC_TEST_DUTY6        (2700)
#define LEDC_TEST_DUTY7        (3150)
#define LEDC_TEST_DUTY8        (3600)
#define LEDC_TEST_DUTY9        (4000)

#define BUF_SIZE        (1024)

static void echo_task(void *arg)
{

  uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity    = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_param_config(UART_NUM_0, &uart_config);
  uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(UART_NUM_0, 2048, 0, 0, NULL, 0);

  
  uint8_t *inputnum = (uint8_t *) malloc(BUF_SIZE);

   
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .freq_hz = 5000,                      // frequency of PWM signal
        .speed_mode = LEDC_HS_MODE,           // timer mode
        .timer_num = LEDC_HS_TIMER            // timer index
    };
    
    ledc_timer_config(&ledc_timer);

    
    ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
        {
            .channel    = LEDC_HS_CH0_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH0_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .timer_sel  = LEDC_HS_TIMER
        },
    };
        ledc_channel_config(&ledc_channel[0]);
    
    while (1) {
    
      int len = uart_read_bytes(UART_NUM_0, inputnum, BUF_SIZE, 20 / portTICK_RATE_MS);
      uart_write_bytes(UART_NUM_0, (const char *) inputnum, len);
        

if (len > 0){
        if (inputnum[0] == '0'){
        ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, LEDC_TEST_DUTY0 );
        ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
          }
          else if(inputnum[0] == '1'){
            ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, LEDC_TEST_DUTY1 );
            ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
          }
          else if(inputnum[0] == '2'){
            ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, LEDC_TEST_DUTY2 );
            ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
          }
          else if(inputnum[0] =='3') {
            ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, LEDC_TEST_DUTY3 );
            ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
          }
          else if(inputnum[0] == '4'){
            ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, LEDC_TEST_DUTY4 );
            ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
          }
          else if(inputnum[0] =='5' ) {
              ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, LEDC_TEST_DUTY5 );
              ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
            }
        else if(inputnum[0] =='6' ) {
            ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, LEDC_TEST_DUTY6 );
            ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
            }
        else if(inputnum[0] =='7' ) {
            ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, LEDC_TEST_DUTY7 );
            ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
            }
        else if(inputnum[0] =='8' ) {
            ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, LEDC_TEST_DUTY8 );
            ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
            }
        else if(inputnum[0] =='9' ) {
            ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, LEDC_TEST_DUTY9 );
            ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
            }
    printf("\n");
             }
    }
}

void app_main()
{
    xTaskCreate(echo_task, "uart_echo_task", 1024, NULL, 5, NULL);
}
