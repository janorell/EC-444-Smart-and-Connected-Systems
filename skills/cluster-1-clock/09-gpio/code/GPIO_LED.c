
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

/* Can use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

#define LED_Blue 26
#define LED_Yellow 25
#define LED_Green 4
#define LED_Red 21

void app_main(void)
{
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    gpio_pad_select_gpio(LED_Blue);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(LED_Blue, GPIO_MODE_OUTPUT);
    
    gpio_pad_select_gpio(LED_Yellow);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(LED_Yellow, GPIO_MODE_OUTPUT);
    
    
    gpio_pad_select_gpio(LED_Green);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(LED_Green, GPIO_MODE_OUTPUT);
    
    gpio_pad_select_gpio(LED_Red);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(LED_Red, GPIO_MODE_OUTPUT);
    while(1) {
        /* Blink off (output low) */
	//printf("Turning off the LED\n");
        
        gpio_set_level(LED_Blue, 0); //off
        gpio_set_level(LED_Yellow, 0);
        gpio_set_level(LED_Green, 0);
        gpio_set_level(LED_Red, 0);
        
        //1
        printf("1\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(LED_Blue, 1); //on
        
        //2
        printf("2\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(LED_Blue, 0);
        gpio_set_level(LED_Yellow, 1);
        
        //3
        printf("3\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(LED_Blue, 1);
        gpio_set_level(LED_Yellow, 1);
        
        //4
        printf("4\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(LED_Blue, 0);
        gpio_set_level(LED_Yellow, 0);
        gpio_set_level(LED_Green, 1);
        
        //5
        printf("5\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(LED_Blue, 1);
        gpio_set_level(LED_Yellow, 0);
        gpio_set_level(LED_Green, 1);
        
        //6
        printf("6\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(LED_Blue, 0);
        gpio_set_level(LED_Yellow, 1);
        gpio_set_level(LED_Green, 1);
        
        //7
        printf("7\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(LED_Blue, 1);
        gpio_set_level(LED_Yellow, 1);
        gpio_set_level(LED_Green, 1);
        
        //8
        printf("8\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(LED_Blue, 0);
        gpio_set_level(LED_Yellow, 0);
        gpio_set_level(LED_Green, 0);
        gpio_set_level(LED_Red, 1);
        
        //9
        printf("9\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(LED_Blue, 1);
        gpio_set_level(LED_Yellow, 0);
        gpio_set_level(LED_Green, 0);
        gpio_set_level(LED_Red, 1);
        
        //10
        printf("10\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(LED_Blue, 0);
        gpio_set_level(LED_Yellow, 1);
        gpio_set_level(LED_Green, 0);
        gpio_set_level(LED_Red, 1);
        
        //11
        printf("11\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(LED_Blue, 1);
        gpio_set_level(LED_Yellow, 1);
        gpio_set_level(LED_Green, 0);
        gpio_set_level(LED_Red, 1);
        
        //12
        printf("12\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(LED_Blue, 0);
        gpio_set_level(LED_Yellow, 0);
        gpio_set_level(LED_Green, 1);
        gpio_set_level(LED_Red, 1);
        
        //13
        printf("13\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(LED_Blue, 1);
        gpio_set_level(LED_Yellow, 0);
        gpio_set_level(LED_Green, 1);
        gpio_set_level(LED_Red, 1);
        
        //14
        printf("14\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(LED_Blue, 0);
        gpio_set_level(LED_Yellow, 1);
        gpio_set_level(LED_Green, 1);
        gpio_set_level(LED_Red, 1);
        
        //15
        printf("15\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(LED_Blue, 1);
        gpio_set_level(LED_Yellow, 1);
        gpio_set_level(LED_Green, 1);
        gpio_set_level(LED_Red, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        
    }
}
