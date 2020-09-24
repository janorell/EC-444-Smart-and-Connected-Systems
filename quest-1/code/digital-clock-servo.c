#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "esp_attr.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#define TIMER_DIVIDER         16    //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // to seconds
#define TIMER_INTERVAL_SEC   (1)    // Sample test interval for the first timer
#define TEST_WITH_RELOAD      1     // Testing will be done with auto reload

// 14-Segment Display
#define SLAVE_ADDR                         0x70 // alphanumeric address
#define OSC                                0x21 // oscillator cmd
#define HT16K33_BLINK_DISPLAYON            0x01 // Display on cmd
#define HT16K33_BLINK_OFF                  0    // Blink off cmd
#define HT16K33_BLINK_CMD                  0x80 // Blink cmd
#define HT16K33_CMD_BRIGHTNESS             0xE0 // Brightness cmd

// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0xFF // i2c nack value

//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH 500 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2400 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 60 //maximum angle in degree upto which servo can rotate
#define SECONDS_GPIO 21
#define MINUTES_GPIO 13

unsigned int secCount;
unsigned int minCount;
unsigned int hourCount;
int flag = 0; //for ticking

static const uint16_t alphafonttable[] = {
    0b0000000000000000, //  space
    0b0000000000000110, // !
    0b0000001000100000, // "
    0b0001001011001110, // #
    0b0001001011101101, // $
    0b0000110000100100, // %
    0b0010001101011101, // &
    0b0000010000000000, // '
    0b0010010000000000, // (
    0b0000100100000000, // )
    0b0011111111000000, // *
    0b0001001011000000, // +
    0b0000100000000000, // ,
    0b0000000011000000, // -
    0b0000000000000000, // .
    0b0000110000000000, // /
    0b0000110000111111, // 0
    0b0000000000000110, // 1
    0b0000000011011011, // 2
    0b0000000010001111, // 3
    0b0000000011100110, // 4
    0b0010000001101001, // 5
    0b0000000011111101, // 6
    0b0000000000000111, // 7
    0b0000000011111111, // 8
    0b0000000011101111, // 9
    0b0001001000000000, // :
    0b0000101000000000, // ;
    0b0010010000000000, // <
    0b0000000011001000, // =
    0b0000100100000000, // >
    0b0001000010000011, // ?
    0b0000001010111011, // @
    0b0000000011110111, // A
    0b0001001010001111, // B
    0b0000000000111001, // C
    0b0001001000001111, // D
    0b0000000011111001, // E
    0b0000000001110001, // F
    0b0000000010111101, // G
    0b0000000011110110, // H
    0b0001001000000000, // I
    0b0000000000011110, // J
    0b0010010001110000, // K
    0b0000000000111000, // L
    0b0000010100110110, // M
    0b0010000100110110, // N
    0b0000000000111111, // O
    0b0000000011110011, // P
    0b0010000000111111, // Q
    0b0010000011110011, // R
    0b0000000011101101, // S
    0b0001001000000001, // T
    0b0000000000111110, // U
    0b0000110000110000, // V
    0b0010100000110110, // W
    0b0010110100000000, // X
    0b0001010100000000, // Y
    0b0000110000001001, // Z
    0b0000000000111001, // [
    0b0010000100000000, //
    0b0000000000001111, // ]
    0b0000110000000011, // ^
    0b0000000000001000, // _
    0b0000000100000000, // `
    0b0001000001011000, // a
    0b0010000001111000, // b
    0b0000000011011000, // c
    0b0000100010001110, // d
    0b0000100001011000, // e
    0b0000000001110001, // f
    0b0000010010001110, // g
    0b0001000001110000, // h
    0b0001000000000000, // i
    0b0000000000001110, // j
    0b0011011000000000, // k
    0b0000000000110000, // l
    0b0001000011010100, // m
    0b0001000001010000, // n
    0b0000000011011100, // o
    0b0000000101110000, // p
    0b0000010010000110, // q
    0b0000000001010000, // r
    0b0010000010001000, // s
    0b0000000001111000, // t
    0b0000000000011100, // u
    0b0010000000000100, // v
    0b0010100000010100, // w
    0b0010100011000000, // x
    0b0010000000001100, // y
    0b0000100001001000, // z
    0b0000100101001001, // {
    0b0001001000000000, // |
    0b0010010010001001, // }
    0b0000010100100000, // ~
    0b0011111111111111,
};

// Function to initiate i2c -- note the MSB declaration!
static void i2c_example_master_init() {
    // Debug
    printf("\n>> i2c Config\n");
    int err;
    
    // Port configuration
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    
    /// Define I2C configurations
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;                              // Master mode
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
    err = i2c_param_config(i2c_master_port, &conf);           // Configure
    if (err == ESP_OK) { printf("- parameters: ok\n"); }
    
    // Install I2C driver
    err = i2c_driver_install(i2c_master_port, conf.mode,
                             I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                             I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    // i2c_set_data_mode(i2c_master_port,I2C_DATA_MODE_LSB_FIRST,I2C_DATA_MODE_LSB_FIRST);
    if (err == ESP_OK) { printf("- initialized: yes\n\n"); }
    
    // Dat in MSB mode
    i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// Utility function to test for I2C device address -- not used in deploy
int testConnection(uint8_t devAddr, int32_t timeout) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

// Utility function to scan for i2c device
static void i2c_scanner() {
    int32_t scanTimeout = 1000;
    printf("\n>> I2C scanning ..."  "\n");
    uint8_t count = 0;
    for (uint8_t i = 1; i < 127; i++) {
        // printf("0x%X%s",i,"\n");
        if (testConnection(i, scanTimeout) == ESP_OK) {
            printf("- Device found at address: 0x%X%s", i, "\n");
            count++;
        }
    }
    if (count == 0)
        printf("- No I2C devices found!" "\n");
    printf("\n");
}

/////////////////////// Alphanumeric Functions /////////////////////////////
// Turn on oscillator for alpha display
int alpha_oscillator() {
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, OSC, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(200 / portTICK_RATE_MS);
    return ret;
}

// Set blink rate to off
int no_blink() {
    int ret;
    i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
    i2c_master_start(cmd2);
    i2c_master_write_byte(cmd2, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd2, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (HT16K33_BLINK_OFF << 1), ACK_CHECK_EN);
    i2c_master_stop(cmd2);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd2);
    vTaskDelay(200 / portTICK_RATE_MS);
    return ret;
}

// Set Brightness
int set_brightness_max(uint8_t val) {
    int ret;
    i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
    i2c_master_start(cmd3);
    i2c_master_write_byte(cmd3, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd3, HT16K33_CMD_BRIGHTNESS | val, ACK_CHECK_EN);
    i2c_master_stop(cmd3);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd3, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd3);
    vTaskDelay(200 / portTICK_RATE_MS);
    return ret;
}

static void mcpwm_example_gpio_initialize() {
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SECONDS_GPIO);    //Set GPIO 4 as PWM0A, to which servo is connected
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MINUTES_GPIO);    //Set GPIO 4 as PWM0A, to which servo is connected
}

/**
 * @brief Use this function to calcute pulse width for per degree rotation
 *
 * @param  degree_of_rotation the angle in degree to which servo has to rotate
 *
 * @return
 *     - calculated pulse width
 */
static uint32_t servo_per_degree_init_MIN(uint32_t MIN_degree_of_rotation) {
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (MIN_degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}
static uint32_t servo_per_degree_init_SEC(uint32_t SEC_degree_of_rotation) {
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (SEC_degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}
/**
 * @brief Configure MCPWM module
 */
void mcpwm_example_servo_control() {
    uint32_t angle_min, angle_sec;
    //1. mcpwm gpio initialization
    
    printf("Angle of rotation: %d\n", secCount);
    angle_sec = servo_per_degree_init_SEC(secCount);
    angle_min = servo_per_degree_init_MIN(minCount);
    printf("pulse width: %dus\n", angle_sec);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle_sec);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle_min);
    
    //while (1) {
    //    for (count = 0; count < SERVO_MAX_DEGREE; count++) {
    //        printf("Angle of rotation: %d\n", count);
    //        angle_sec = servo_per_degree_init_SEC(count);
    //        angle_min = servo_per_degree_init_MIN(count / 60);
    //        printf("pulse width: %dus\n", angle_sec);
    //        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle_sec);
    //        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle_min);
    //        vTaskDelay(10);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
    //    }
    //}
}

////////////////////////////////////////////////////////////////////////////////

void IRAM_ATTR timer_isr() {
    TIMERG0.int_clr_timers.t0 = 1;  // clear timer
    flag = 1;
}

static void timer_initialize() {
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = TEST_WITH_RELOAD;
    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    
    // Timer's counter will initially start from value below
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
    
    // Configure the alarm value and the interrupt on alarm
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_SEC * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_isr, (void*)TIMER_0, ESP_INTR_FLAG_IRAM, NULL);
    
    // Start timer
    timer_start(TIMER_GROUP_0, TIMER_0);
}

void writeTimeToDisplay(char hourStr[20], char minStr[20]) {
    int hour_first;
    int hour_second;
    int min_first;
    int min_second;
    
    // Debug
    int ret;
    printf(">> Test Alphanumeric Display: \n");
    
    // Set up routines
    // Turn on alpha oscillator
    ret = alpha_oscillator();
    if(ret == ESP_OK) {
        printf("- oscillator: ok \n");
    }
    // Set display blink off
    ret = no_blink();
    if(ret == ESP_OK) {
        printf("- blink: off \n");
    }
    ret = set_brightness_max(0xF);
    if(ret == ESP_OK) {
        printf("- brightness: max \n");
    }
    
    if(strlen(hourStr) < 2){
        hour_first = 16;
        hour_second = hourStr[0] - 32;
    } else {
        hour_first = hourStr[0] - 32;
        hour_second = hourStr[1] - 32;
    }
    
    if(strlen(minStr) < 2){
        min_first = 16;
        min_second = minStr[0] - 32;
    } else {
        min_first = minStr[0] - 32;
        min_second = minStr[1] - 32;
    }
    
    // Write to characters to buffer
    uint16_t displaybuffer[8];
    displaybuffer[0] = alphafonttable[hour_first];
    displaybuffer[1] = alphafonttable[hour_second];
    displaybuffer[2] = alphafonttable[min_first];
    displaybuffer[3] = alphafonttable[min_second];
    
    // Send commands characters to display over I2C
    i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
    i2c_master_start(cmd4);
    i2c_master_write_byte(cmd4, (SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
    for (uint8_t i=0; i<8; i++) {
        i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
        i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
    }
    i2c_master_stop(cmd4);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd4);
}

void tickTime() {
    // starts at 0 so 1 less than each max
    if (secCount > 58) {
        secCount = 0;
        minCount++;
        if (minCount > 58) {
            minCount = 0;
            hourCount++;
            if (hourCount > 23) {
                hourCount = 0;
            }
        }
        char hour_int[20];
        char min_int[20];
        sprintf(hour_int, "%d", hourCount);
        sprintf(min_int, "%d", minCount);
        writeTimeToDisplay(hour_int, min_int);
        mcpwm_example_servo_control();
        // mcpwm_example_servo_control(minCount, minute_servo);
        
    }
    else {
        mcpwm_example_servo_control();
        // mcpwm_example_servo_control(secCount, second_servo);
        secCount++;
    }
}

void app_main(void) {
    i2c_example_master_init();
    i2c_scanner();
    
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));
    esp_vfs_dev_uart_use_driver(UART_NUM_0);
    
    mcpwm_example_gpio_initialize();
    
    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm......\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    
    //xTaskCreate(mcpwm_example_servo_control, "mcpwm_example_servo_control", 4096, NULL, 5, NULL);
    
    flag = 0;
    
    char hourStr[20];
    char minStr[20];
    char secStr[20];
    
    //int hour_first;
    //int hour_second;
    //int min_first;
    //int min_second;
    
    char temp;
    
    // Debug
    // int ret;
    // printf(">> Test Alphanumeric Display: \n");
    
    // // Set up routines
    // // Turn on alpha oscillator
    // ret = alpha_oscillator();
    // if(ret == ESP_OK) {printf("- oscillator: ok \n");}
    // // Set display blink off
    // ret = no_blink();
    // if(ret == ESP_OK) {printf("- blink: off \n");}
    // ret = set_brightness_max(0xF);
    // if(ret == ESP_OK) {printf("- brightness: max \n");}
    
    // Initiate timer
    printf("Enter hour in HH: ");              //hour
    scanf("%[^\n]", hourStr);
    printf("%s\n", hourStr);
    hourCount = atoi(hourStr);
    
    while(hourCount > 23 || strlen(hourStr) > 2 || (hourStr[1] < 48 && hourStr[1] != 0) || hourStr[1] > 57 || hourStr[0] < 48 || hourStr[0] > 57){
        printf("Invalid input: %s is not a valid hour\n", hourStr);
        printf("Enter hour in HH: ");
        scanf("%c", &temp); //temp to clear buffer
        scanf("%[^\n]", hourStr);
        printf("%s\n", hourStr);
        hourCount = atoi(hourStr);
    }
    
    printf("Enter minute in MM: ");             //min
    //scanf("%c", &temp); //temp to clear buffer
    scanf("%s", minStr);
    printf("%s\n", minStr);
    minCount = atoi(minStr);
    
    while(minCount > 59 || strlen(minStr) > 2 || (minStr[1] < 48 && minStr[1] != 0) || minStr[1] > 57 || minStr[0] < 48 || minStr[0] > 57){
        printf("Invalid input: %s is not a valid minute\n", minStr);
        printf("Enter minute in MM: ");
        scanf("%c", &temp); //temp to clear buffer
        scanf("%[^\n]", minStr);
        printf("%s\n", minStr);
        minCount = atoi(minStr);
    }
    
    printf("Enter second in SS: ");             //sec
    //scanf("%c", &temp); //temp to clear buffer
    scanf("%s", secStr);
    printf("%s\n", secStr);
    secCount = atoi(secStr);
    
    while(secCount > 59 || strlen(secStr) > 2 || (secStr[1] < 48 && secStr[1] != 0) || secStr[1] > 57 || secStr[0] < 48 || secStr[0] > 57){
        printf("Invalid input: %s is not a valid second\n", secStr);
        printf("Enter second in SS: ");
        scanf("%c", &temp); //temp to clear buffer
        scanf("%[^\n]", secStr);
        printf("%s\n", secStr);
        secCount = atoi(secStr);
    }
    
    // hour_first = hourStr[0] - 32;
    // hour_second = hourStr[1] - 32;
    // min_first = minStr[0] - 32;
    // min_second = minStr[1] - 32;
    
    // // Write to characters to buffer
    // uint16_t displaybuffer[8];
    // displaybuffer[0] = alphafonttable[hour_first];
    // displaybuffer[1] = alphafonttable[hour_second];
    // displaybuffer[2] = alphafonttable[min_first];
    // displaybuffer[3] = alphafonttable[min_second];
    
    // everything is string -> convert to int (for counter purposes and error check)
    // hourCount = atoi(hourStr);
    // minCount = atoi(minStr);
    // secCount = atoi(secStr);
    
    timer_initialize();
    
    writeTimeToDisplay(hourStr, minStr);
    mcpwm_example_servo_control();
    
    // hour_first = hourStr[0] - 32;
    // hour_second = hourStr[1] - 32;
    // min_first = minStr[0] - 32;
    // min_second = minStr[1] - 32;
    
    // // Write to characters to buffer
    // uint16_t displaybuffer[8];
    // displaybuffer[0] = alphafonttable[hour_first];
    // displaybuffer[1] = alphafonttable[hour_second];
    // displaybuffer[2] = alphafonttable[min_first];
    // displaybuffer[3] = alphafonttable[min_second];
    
    // everything is string -> convert to int (for counter purposes and error check)
    hourCount = atoi(hourStr);
    minCount = atoi(minStr);
    secCount = atoi(secStr);
    
    timer_initialize();
    
    // // Send commands characters to display over I2C
    //       i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
    //       i2c_master_start(cmd4);
    //       i2c_master_write_byte(cmd4, (SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    //       i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
    //       for (uint8_t i=0; i<8; i++) {
    //         i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
    //         i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
    //       }
    //       i2c_master_stop(cmd4);
    //       ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_RATE_MS);
    //       i2c_cmd_link_delete(cmd4);
    
    while (1) {
        if (flag == 1) {
            //printf("one second- stuff\n");
            tickTime();
            printf("sec: %u\n", secCount);
            printf("min: %u\n", minCount);
            printf("hr: %u\n\n", hourCount);
            flag = 0;
            
            // After the alarm triggers, we need to re-enable it to trigger it next time
            TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // delay to keeps from idling
    }
}
