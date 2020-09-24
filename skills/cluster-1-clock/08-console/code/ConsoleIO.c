#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

// #define ECHO_TEST_TXD  (GPIO_NUM_1)
// #define ECHO_TEST_RXD  (GPIO_NUM_3)
// #define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
// #define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)

// #define BUF_SIZE (1024)

#define GPIO 13

void app_main() {
    ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0) );
    esp_vfs_dev_uart_use_driver(UART_NUM_0);
    
    gpio_pad_select_gpio(GPIO);
    gpio_set_direction(GPIO, GPIO_MODE_OUTPUT);
    
    int mode = 1;
    
    char switchz[2];
    char tog[2];
    int ledtog = 0;
    char rem;
    
    strcpy(switchz, "s");
    strcpy(tog, "t");
    
    char string[50];
    while(1) {
        switch (mode) {
                /////
            case 1:
                printf("toggle mode\n");
                while(1) {
                    scanf("%s", string);
                    printf("Read: %s\n", string);
                    //if ( string[0] != 't' || string[0] != 's')
                    //    printf("not a valid input --- ");
                    
                    if (strcmp(string, switchz) == 0) {
                        mode = 2;
                        gpio_set_level(GPIO, 0);
                        ledtog = 0;
                        
                        vTaskDelay(50/portTICK_RATE_MS);
                        break;
                        // toggle
                    } else if (strcmp (string, tog) == 0){
                        if (ledtog == 1) {
                            gpio_set_level(GPIO, 0);
                            ledtog = 0;
                            
                        } else {
                            gpio_set_level(GPIO, 1);
                            ledtog = 1;
                        }
                    }
                    
                }
                
            case 2:
                printf("echo mode - enter a string: \n");
                while(1){
                    scanf("%c", &rem);
                    scanf("%[^\n]", string);
                    printf("Echo: %s\n", string);
                    if (strcmp (switchz, string) == 0) {
                        mode = 3;
                        vTaskDelay(50 / portTICK_RATE_MS);
                        break;
                    }
                }
                
            case 3:
                printf("dec to hex\n");
                while(1){
                    long int dec, q;
                    int i = 1, j, rem;
                    char hexadec[100];
                    printf("Enter an integer:\n");
                    scanf("%s", string);
                    if (strcmp (switchz, string) == 0) {
                        mode = 1;
                        vTaskDelay(50 / portTICK_RATE_MS);
                        break;
                        
                        
                        
                    } else {
                        dec = atoi(string); // string to int
                    }
                    q = dec;
                    while (q!=0) {
                        rem = q % 16;
                        if (rem < 10) { 
                            rem = rem + 48;
                        } else {
                            rem = rem + 55;
                        }
                        hexadec[i++]= rem;
                        q = q / 16;
                    }
                    printf("Hex: ");
                    for (j = i -1; j > 0; j--) {
                        printf("%c", hexadec[j]);
                    }
                    printf("\n");
                }
        }
    }
}
