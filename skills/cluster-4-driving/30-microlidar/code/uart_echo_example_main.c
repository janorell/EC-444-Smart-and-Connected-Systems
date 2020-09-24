
#include <stdio.h>
#include <string.h>
#include "esp_vfs_dev.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "soc/uart_struct.h"
#include "string.h"


#define RX (16)
#define TX (17)

static const int BUF_SIZE = 256;

static void echo_task()
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TX, RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);


    while(1){
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    uint32_t distance = 0;

    int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 100);
    uart_flush(UART_NUM_1);
    
   // for (int i =0; i < len; i++){
        if (data[0] == 0x59 && data[1] == 0x59)
        {
            
            distance = (data[3]<<8)+data[2];
            
       // }
    }
        printf("Distance: %dcm\n", distance);
    }
}


void app_main()
{
    xTaskCreate(echo_task, "uart_echo_task", 1024, NULL, 10, NULL);
}
