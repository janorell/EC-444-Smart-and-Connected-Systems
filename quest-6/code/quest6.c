/*
  - Quest6
  - Code to run crawler

  Team7: Vanessa Schuweh, Vindhya Kuchibhotla, Jennifer Norell
  December 10, 2019
*/
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <sys/param.h>
#include <time.h>
#include "./ADXL343.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/portmacro.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_adc_cal.h"
#include "esp_vfs_dev.h"
#include "esp_types.h"

#include "driver/mcpwm.h"
#include "driver/i2c.h"
#include "driver/rmt.h"
#include "driver/uart.h"
#include "driver/periph_ctrl.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/timer.h"
#include "driver/ledc.h"
#include "driver/pcnt.h"

#include "soc/rmt_reg.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

//Range sensors
int distCollision; //Lidar
int driveSpeed = 1200;
double distance_IR1;
double distance_IR2;
double distance_IR3;

//Split
unsigned int sec = 0;
unsigned int min = 0;

//Manual vs Auto
int isAuto = -1;
int manualDrive = 1400;
int manualTurn = 1400;

//Define tasks
xTaskHandle task_rec;
xTaskHandle task_led;
xTaskHandle task_ir;
xTaskHandle task_coll;
xTaskHandle task_speed;
static void recieve_task();
static void led_task();
static void collision_task();
static void ir_task();

//PID speed
#define setpoint 0.2 //speed for PID to maintain in m/s
#define Kp 0.75
#define Ki 0.5
#define Kd 0.0
double derivative, error, output;
double dt = 0.1;
double prev_error = 0.0; // Set up PID loop
double integral = 0.0;
double speed;

// Servo control /////////////////////////////////////////////////////////////////////////////////
//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH 1000 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2000 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 90 //Maximum angle in degree upto which servo can rotate

void mcpwm_initialize() {
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 4);    //Set GPIO 4 as PWM0A (A5) , to which ESC is connected
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, 21);    //Set GPIO 21 as PWM1A (21), to which steering servo is connected

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm......\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);
}

//Function to control motor forward and backward
void motorDrive(int speedVal) { 
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, speedVal);
}

//Funtion to control motor turn
void motorTurn(int turnVal){
    mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, turnVal);
}
//////////////////////////////////////////////////////////////////////////////////////////////////

// IR //////////////////////////////////////////////////////////////////////////////////////////////
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t ir1_channel = ADC_CHANNEL_0;     //GPIO34 (A2- 6), (A4- 0) if ADC1, GPIO14 if ADC2
static const adc_channel_t ir2_channel = ADC_CHANNEL_3;     //GPIO39 (A3) if ADC1, GPIO14 if ADC2
static const adc_channel_t ir3_channel = ADC_CHANNEL_5;     //GPIO33 (33) if ADC1, GPIO14 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_11; // attenuation - db 11 (for full range)
static const adc_unit_t unit = ADC_UNIT_1;

static void check_efuse(){
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type){
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

void adc_config(){
    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(ir1_channel, atten);
        adc1_config_channel_atten(ir2_channel, atten);
        adc1_config_channel_atten(ir3_channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)ir1_channel, atten);
        adc2_config_channel_atten((adc2_channel_t)ir2_channel, atten);
        adc2_config_channel_atten((adc2_channel_t)ir3_channel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);
}

void ir_task(){ //task to read IR sensor values
    while (1) {
        double volt_IR1;
        double volt_IR2;
        double volt_IR3;
        uint32_t adc_reading_IR1 = 0;
        uint32_t adc_reading_IR2 = 0;
        uint32_t adc_reading_IR3 = 0;

        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading_IR1 += adc1_get_raw((adc1_channel_t)ir1_channel);
                adc_reading_IR2 += adc1_get_raw((adc1_channel_t)ir2_channel);
                adc_reading_IR3 += adc1_get_raw((adc1_channel_t)ir3_channel);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)ir1_channel, ADC_WIDTH_BIT_12, &raw);
                adc_reading_IR1 += raw;
            }
        }
        adc_reading_IR1 /= NO_OF_SAMPLES;
        adc_reading_IR2 /= NO_OF_SAMPLES;
        adc_reading_IR3 /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
        uint32_t voltage_IR1 = esp_adc_cal_raw_to_voltage(adc_reading_IR1, adc_chars);
        uint32_t voltage_IR2 = esp_adc_cal_raw_to_voltage(adc_reading_IR2, adc_chars);
        uint32_t voltage_IR3 = esp_adc_cal_raw_to_voltage(adc_reading_IR3, adc_chars);

        volt_IR1 = (double)voltage_IR1 / 1000;
        volt_IR2 = (double)voltage_IR2 / 1000;
        volt_IR3 = (double)voltage_IR3 / 1000;

        //Get IR values from polynomial regression(same from IR range finder skill)
        if (volt_IR1 > 1) {
            distance_IR1 = (double)(volt_IR1 - 3.316) / -0.0431;
        } else if (1 > volt_IR1 || volt_IR1 < 0.5) {
            distance_IR1 = (double)(volt_IR1 - 1.5211) / -0.0099;
        } else {
            distance_IR1 = (double)(volt_IR1 - 1.3335) / -0.0078;
        }
        distance_IR1 = (double)distance_IR1 / 100;

        if (volt_IR2 > 1) {
            distance_IR2 = (double)(volt_IR2 - 3.316) / -0.0431;
        } else if (1 > volt_IR2 || volt_IR2 < 0.5) {
            distance_IR2 = (double)(volt_IR2 - 1.5211) / -0.0099;
        } else {
            distance_IR2 = (double)(volt_IR2 - 1.3335) / -0.0078;
        }
        distance_IR2 = (double)distance_IR2 / 100;

        //third IR for backwards collision prevention
        if (volt_IR3 > 1) {
            distance_IR3 = (double)(volt_IR3 - 3.316) / -0.0431;
        } else if (1 > volt_IR3 || volt_IR3 < 0.5) {
            distance_IR3 = (double)(volt_IR3 - 1.5211) / -0.0099;
        } else {
            distance_IR3 = (double)(volt_IR3 - 1.3335) / -0.0078;
        }
        distance_IR3 = (double)distance_IR3 / 100;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////

// ALPHANUMERIC /////////////////////////////////////////////////////////////////////////////////
// 14-Segment Display for Alphanumeric
#define SLAVE_ADDR_ALPHA                   0x70 // alphanumeric address
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

static const uint16_t alphafonttable[] = { //table of alphanumeric codes
    0b0000000000000001,
    0b0000000000000010,
    0b0000000000000100,
    0b0000000000001000,
    0b0000000000010000,
    0b0000000000100000,
    0b0000000001000000,
    0b0000000010000000,
    0b0000000100000000,
    0b0000001000000000,
    0b0000010000000000,
    0b0000100000000000,
    0b0001000000000000,
    0b0010000000000000,
    0b0100000000000000,
    0b1000000000000000,
    0b0000000000000000,
    0b0000000000000000,
    0b0000000000000000,
    0b0000000000000000,
    0b0000000000000000,
    0b0000000000000000,
    0b0000000000000000,
    0b0000000000000000,
    0b0001001011001001,
    0b0001010111000000,
    0b0001001011111001,
    0b0000000011100011,
    0b0000010100110000,
    0b0001001011001000,
    0b0011101000000000,
    0b0001011100000000,
    0b0000000000000000, //
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
    0b0100000000000000, // .
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

int alpha_oscillator() {
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR_ALPHA << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, OSC, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(200 / portTICK_RATE_MS);
    return ret;
}

int no_blink() {
    int ret;
    i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
    i2c_master_start(cmd2);
    i2c_master_write_byte(cmd2, ( SLAVE_ADDR_ALPHA << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd2, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (HT16K33_BLINK_OFF << 1), ACK_CHECK_EN);
    i2c_master_stop(cmd2);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd2);
    vTaskDelay(200 / portTICK_RATE_MS);
    return ret;
}

int set_brightness_max(uint8_t val) {
    int ret;
    i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
    i2c_master_start(cmd3);
    i2c_master_write_byte(cmd3, ( SLAVE_ADDR_ALPHA << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd3, HT16K33_CMD_BRIGHTNESS | val, ACK_CHECK_EN);
    i2c_master_stop(cmd3);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd3, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd3);
    vTaskDelay(200 / portTICK_RATE_MS);
    return ret;
}

void writeTimeToDisplay(char secStr[5], char minStr[5]) {
    int sec_first;
    int sec_second;
    int min_first;
    int min_second;

    printf("secStr: %s\n", secStr);
    printf("minStr: %s\n", minStr);

    if(strlen(secStr) < 2){
        sec_first = 16;
        sec_second = secStr[0] - 32;
    } else {
        sec_first = secStr[0] - 32;
        sec_second = secStr[1] - 32;
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
    displaybuffer[0] = alphafonttable[min_first];
    displaybuffer[1] = alphafonttable[min_second];
    displaybuffer[2] = alphafonttable[sec_first];
    displaybuffer[3] = alphafonttable[sec_second];

    // Send commands characters to display over I2C
    i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
    i2c_master_start(cmd4);
    i2c_master_write_byte(cmd4, (SLAVE_ADDR_ALPHA << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
    for (uint8_t i=0; i<8; i++) {
        i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
        i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
    }
    i2c_master_stop(cmd4);
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd4);
}
////////////////////////////////////////////////////////////////////////////////////////////////

// Speed ///////////////////////////////////////////////////////////////////////////////////////////
#define PCNT_TEST_UNIT      PCNT_UNIT_0
#define PCNT_H_LIM_VAL      10000
#define PCNT_L_LIM_VAL     -10
#define PCNT_THRESH1_VAL    10000
#define PCNT_THRESH0_VAL   -5
#define PCNT_INPUT_SIG_IO   34  // A2
#define PCNT_INPUT_CTRL_IO  5  

#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL0_SEC   (1.0) // sample test interval for the first timer
#define TIMER_INTERVAL1_SEC   (1.0)   // sample test interval for the second timer
#define TEST_WITHOUT_RELOAD   0        // testing will be done without auto reload
#define TEST_WITH_RELOAD      1        // testing will be done with auto reload
#define DEFAULT_VREF          1100        //Use adc2_vref_to_gpio() to obtain a better estimate

xQueueHandle pcnt_evt_queue;   // A queue to handle pulse counter events
pcnt_isr_handle_t user_isr_handle = NULL; //user's ISR service handle
int16_t count = 0;
int16_t prevCount = 0; 
/* A sample structure to pass events from the PCNT
 * interrupt handler to the main program.
 */
typedef struct {
    int unit;  
    uint32_t status;
} pcnt_evt_t;

/*
 * A sample structure to pass events
 * from the timer interrupt handler to the main program.
 */
typedef struct {
    int type;  // the type of timer's event
    int timer_group;
    int timer_idx;
    uint64_t timer_counter_value;
} timer_event_t;

xQueueHandle timer_queue;

/*
 * Timer group0 ISR handler
 *
 * Note:
 * We don't call the timer API here because they are not declared with IRAM_ATTR.
 * If we're okay with the timer irq not being serviced while SPI flash cache is disabled,
 * we can allocate this interrupt without the ESP_INTR_FLAG_IRAM flag and use the normal API.
 */
void IRAM_ATTR timer_group0_isr(void *para){
    int timer_idx = (int) para;

    /* Retrieve the interrupt status and the counter value
       from the timer that reported the interrupt */
    timer_intr_t timer_intr = timer_group_intr_get_in_isr(TIMER_GROUP_0);
    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, timer_idx);

    /* Prepare basic event data
       that will be then sent back to the main program task */
    timer_event_t evt;
    evt.timer_group = 0;
    evt.timer_idx = timer_idx;
    evt.timer_counter_value = timer_counter_value;

    /* Clear the interrupt
       and update the alarm time for the timer with without reload */
    if (timer_intr & TIMER_INTR_T0) {
        evt.type = TEST_WITHOUT_RELOAD;
        timer_group_intr_clr_in_isr(TIMER_GROUP_0, TIMER_0);
        timer_counter_value += (uint64_t) (TIMER_INTERVAL0_SEC * TIMER_SCALE);
        timer_group_set_alarm_value_in_isr(TIMER_GROUP_0, timer_idx, timer_counter_value);
    } else if (timer_intr & TIMER_INTR_T1) {
        evt.type = TEST_WITH_RELOAD;
        timer_group_intr_clr_in_isr(TIMER_GROUP_0, TIMER_1);
    } else {
        evt.type = -1; // not supported even type
    }

    /* After the alarm has been triggered
      we need enable it again, so it is triggered the next time */
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, timer_idx);

    /* Now just send the event data back to the main program task */
    xQueueSendFromISR(timer_queue, &evt, NULL);
}

static void example_tg0_timer_init(int timer_idx, bool auto_reload, double timer_interval_sec) {
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = auto_reload;
    #ifdef CONFIG_IDF_TARGET_ESP32S2BETA
        config.clk_sel = TIMER_SRC_CLK_APB;
    #endif
        timer_init(TIMER_GROUP_0, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
    timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr,
        (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    timer_start(TIMER_GROUP_0, timer_idx);
}

/* Decode what PCNT's unit originated an interrupt
 * and pass this information together with the event type
 * the main program using a queue.
 */
static void IRAM_ATTR pcnt_example_intr_handler(void *arg){
    uint32_t intr_status = PCNT.int_st.val;
    int i;
    pcnt_evt_t evt;
    portBASE_TYPE HPTaskAwoken = pdFALSE;

    for (i = 0; i < PCNT_UNIT_MAX; i++) {
        if (intr_status & (BIT(i))) {
            evt.unit = i;
            /* Save the PCNT event type that caused an interrupt
               to pass it to the main program */
            evt.status = PCNT.status_unit[i].val;
            PCNT.int_clr.val = BIT(i);
            xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);
            if (HPTaskAwoken == pdTRUE) {
                portYIELD_FROM_ISR();
            }
        }
    }
}

/* Initialize PCNT functions:
 *  - configure and initialize PCNT
 *  - set up the input filter
 *  - set up the counter events to watch
 */
static void pcnt_example_init(void) {
    /* Prepare configuration for the PCNT unit */
    pcnt_config_t pcnt_config = {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = PCNT_INPUT_SIG_IO,
        .ctrl_gpio_num = PCNT_INPUT_CTRL_IO,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_TEST_UNIT,
        // What to do on the positive / negative edge of pulse input?
        .pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
        .neg_mode = PCNT_COUNT_DIS,   // Keep the counter value on the negative edge
        // What to do when control input is low or high?
        .lctrl_mode = PCNT_MODE_REVERSE, // Reverse counting direction if low
        .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high
        // Set the maximum and minimum limit values to watch
        .counter_h_lim = PCNT_H_LIM_VAL,
        .counter_l_lim = PCNT_L_LIM_VAL,
    };
    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);

    /* Configure and enable the input filter */
    pcnt_set_filter_value(PCNT_TEST_UNIT, 100);
    pcnt_filter_enable(PCNT_TEST_UNIT);

    /* Set threshold 0 and 1 values and enable events to watch */
    pcnt_set_event_value(PCNT_TEST_UNIT, PCNT_EVT_THRES_1, PCNT_THRESH1_VAL);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_THRES_1);
    pcnt_set_event_value(PCNT_TEST_UNIT, PCNT_EVT_THRES_0, PCNT_THRESH0_VAL);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_THRES_0);

    /* Enable events on zero, maximum and minimum limit values */
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_ZERO);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_L_LIM);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(PCNT_TEST_UNIT);
    pcnt_counter_clear(PCNT_TEST_UNIT);

    /* Register ISR handler and enable interrupts for PCNT unit */
    pcnt_isr_register(pcnt_example_intr_handler, NULL, 0, &user_isr_handle);
    pcnt_intr_enable(PCNT_TEST_UNIT);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(PCNT_TEST_UNIT);
}

/*
 * The main task of this example program
 */
static void speed_task(void *arg) {
    while (1) {
    timer_event_t evt;
    xQueueReceive(timer_queue, &evt, portMAX_DELAY);
    pcnt_get_counter_value(PCNT_TEST_UNIT, &count);

    //calculate speed from pulse counter
    double pulseCount = count - prevCount;
    prevCount = count; 
    double rotations = (pulseCount/6) * 60/TIMER_INTERVAL1_SEC;  
    float mps = rotations * 0.62 / 60;
    speed = mps; //setting global variable 
    }
}

//Speed control
void pid_speed() {
  error = setpoint - speed;
  dt = 0.1; // important!!!
  integral = integral + error * dt;
  derivative = (error - prev_error) / dt;
  output = Kp * error + Ki * integral + Kd * derivative;
  prev_error = error;
  
  printf("PID speed output: %f\n", output);
  if (output == 0) { //stay the same
    motorDrive(driveSpeed);
  } else if (output < .35) { //slowing down
    if (driveSpeed >= 1300){ //don't let drive slower than 1300. It won't move past that
      driveSpeed = 1300;
    } else {
      driveSpeed += 20; // decrement by 20 if too fast
    }
    motorDrive(driveSpeed); //tell motor to drive
  } else if (output > .35) { //speeding up
    if (driveSpeed <= 1200){ //don't let drive faster than 1200
      driveSpeed = 1200;
    } else {
      driveSpeed = driveSpeed - 20; //increment by 20 if too slow
    }
    motorDrive(driveSpeed);
  } else {
    vTaskDelay(1/portTICK_PERIOD_MS); //do nothing
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////

// Lidar /////////////////////////////////////////////////////////////////////////////////////////
// LIDAR
#define SLAVE_ADDR                         0x62 // 0x62
#define RegisterMeasure                    0x00 // Register to write to initiate ranging.
#define MeasureValue                       0x04 // Value to initiate ranging.
#define RegisterHighLowB                   0x8f // Register to get both High and Low bytes in 1 call.

// Function to initiate i2c -- note the MSB declaration!
static void i2c_master_init(){
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
    if (err == ESP_OK) {printf("- parameters: ok\n");}

    // Install I2C driver
    err = i2c_driver_install(i2c_master_port, conf.mode,
                             I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                             I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    if (err == ESP_OK) {printf("- initialized: yes\n");}

    // Data in MSB mode
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
            printf( "- Device found at address: 0x%X%s", i, "\n");
            count++;
        }
    }
    if (count == 0) {printf("- No I2C devices found!" "\n");}
}

// Get Device ID
int getDeviceID(uint8_t *data) {
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ADXL343_REG_DEVID, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data, ACK_CHECK_DIS);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Write one byte to register
int writeRegister(uint8_t reg, uint8_t data) {
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Read register
uint8_t readRegister(uint8_t reg) {
    uint8_t data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data, ACK_CHECK_DIS);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    printf("data: %x   \n", data);
    return data;
}

// read 16 bits (2 bytes)
int16_t read16(uint8_t reg) {
    uint8_t data1;
    uint8_t data2;
    int16_t data3;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
    i2c_master_start(cmd2);
    i2c_master_write_byte(cmd2, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd2, &data1, ACK_VAL);
    i2c_master_read_byte(cmd2, &data2, ACK_VAL);
    i2c_master_stop(cmd2);
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd2);

    int16_t data2_16 = data1 << 8;
    data3 = data2_16 | data2;
    return data3;
}

//funtion that returns lidar readings
int lidarGetRange() {
    int val = 0;

    writeRegister(RegisterMeasure, MeasureValue);
    vTaskDelay(200 / portTICK_RATE_MS);
    val = read16(RegisterHighLowB);

    printf("Distance: %d\n", val);
    return val;
}

//task to prevent collision in front sensor and turning left
void collision_task(){
	while(1){
        distCollision = lidarGetRange();
        if (distCollision <= 80) { // stop and adjust
            printf("Stopping and adjusting\n");
            motorDrive(1400); //stop
            vTaskDelay(500 / portTICK_PERIOD_MS);
            while (distance_IR3 >= .8 && distCollision <= 90) {// backwards until too close to back wall
                motorTurn(700); //hard right
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                if(driveSpeed != 1400){
                    motorDrive(1550); //reverse slowly
                }
            }
            motorDrive(1400); //stop
            motorTurn(2300); //turn left
            vTaskDelay(1500 / portTICK_PERIOD_MS);
		} else if (distCollision <= 175) { // start turning
            printf("Starting left turn \n");
            motorTurn(2300); // hard left
            motorDrive(driveSpeed);
        } else { // keep 'er straight
            int turnRadius = 1400;
            if (distance_IR1 >= 1.25) { // too far from wall
                printf("Too far from wall, slight right\n");
                turnRadius = 1100;
            } else if (distance_IR1 < .75) { // too close to wall
                printf("Too close to wall, turning left\n");
                turnRadius = 1700;
            } else if (distance_IR1 - distance_IR2 >= .08){ // turn right to straighten
                printf("Straightening right\n");
                turnRadius = 1100;
            } else if (distance_IR1 - distance_IR2 <= -.08){ // turn left to straighten
                printf("Straightening left\n");
                turnRadius = 1700;
            }
            motorTurn(turnRadius);
			printf("Drive\n");
            pid_speed();
    	    motorDrive(driveSpeed);
		}
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////

// hub ///////////////////////////////////////////////////////////////////////////////////////////
// RMT definitions
#define RMT_TX_CHANNEL    1     // RMT channel for transmitter
#define RMT_TX_GPIO_NUM   25    // GPIO number for transmitter signal -- A1
#define RMT_CLK_DIV       100   // RMT counter clock divider
#define RMT_TICK_10_US    (80000000/RMT_CLK_DIV/100000)   // RMT counter value for 10 us.(Source clock is APB clock)
#define rmt_item32_tIMEOUT_US   9500     // RMT receiver timeout value(us)

// UART definitions
#define UART_TX_GPIO_NUM 26 // A0
#define UART_RX_GPIO_NUM 34 // A2
#define BUF_SIZE (1024)

// Hardware interrupt definitions
#define GPIO_INPUT_IO_1       4
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INPUT_PIN_SEL    1ULL<<GPIO_INPUT_IO_1

// LED Output pins definitions
#define GREENLED   14
#define YELLOWLED  32
#define REDLED    15

//wifi config
#ifdef CONFIG_EXAMPLE_IPV4
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#else
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#endif
#define PORT CONFIG_EXAMPLE_PORT

int id = -1;
char color = 'R';
int hubID = 7;
char checksum = 'O';

static const char *TAG = "example";

static void udp_client_task(void *pvParameters) {
    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;
    while (1) {
#ifdef CONFIG_EXAMPLE_IPV4
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
        struct sockaddr_in6 dest_addr;
        inet6_aton(HOST_IP_ADDR, &dest_addr.sin6_addr);
        dest_addr.sin6_family = AF_INET6;
        dest_addr.sin6_port = htons(PORT);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(dest_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);
        char payload[200];
        char split[150];
        while (1) {
            sprintf(split, "%u:%u", min, sec);
            sprintf(payload, "{\"beaconID\":\"%d\",\"splitTime\":\"%s\"}\n", id, split); //sending in JSON
            int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
            ESP_LOGI(TAG, "Message sent: %s", payload);

            struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            } else { // Data received
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);
                if (strcmp(rx_buffer, "1") == 0) {
                    ESP_LOGI(TAG, "Fast"); //if received fast
                    if (manualDrive <= 700){ //can't speed up past 700
                        manualDrive = 700;
                    } else if (manualDrive > 1300){ //can't be between 1400 and 1300 (won't drive)
                        manualDrive = 1300;
                    } else {
                        manualDrive -= 50; //increment speed by 50
                    }
                } else if (strcmp(rx_buffer, "2") == 0) {
                    ESP_LOGI(TAG, "Slow"); //if receive slow
                    if (manualDrive >= 1300 ){ //if lower than 1300 --> stop
                        manualDrive = 1400;
                    } else {
                        manualDrive += 50; //decfrement speed by 50
                    }
                }
                else if (strcmp(rx_buffer, "3") == 0) {
                    ESP_LOGI(TAG, "Left"); //if rceive left
                    if (manualTurn >= 2300 ){ //can't turn left more than 2300
                        manualTurn = 2300;
                    } else {
                        manualTurn += 450; //turn left by 450 (larger step for faster steering)
                    }
                }
                else if (strcmp(rx_buffer, "4") == 0) {
                    ESP_LOGI(TAG, "Right"); //if receive right
                    if (manualTurn <= 700 ){ // can't turn right more than 700 
                        manualTurn = 700;
                    } else {
                        manualTurn -= 450; // turn right by 450
                    }
                }
                else if (strcmp(rx_buffer, "-5") == 0) {
                    ESP_LOGI(TAG, "Auto"); //if rceive auto
                    isAuto = 1; //set flag high
                }
                else if (strcmp(rx_buffer, "5") == 0) {
                    ESP_LOGI(TAG, "Manual");// if receive manual
                    isAuto = 0; //set flag low
                }
            }
            printf("Manual Drive: %d\nManual Turn: %d\n", manualDrive, manualTurn);
            motorDrive(manualDrive); //set new drive and turn vairiables so not to interfere with auto
            motorTurn(manualTurn);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

static void uart_init() {
    uart_config_t uart_config = {
        .baud_rate = 1200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, UART_TX_GPIO_NUM, UART_RX_GPIO_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_set_line_inverse(UART_NUM_1,UART_INVERSE_RXD);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
}

static void led_init() { //led on crawler to indicate the code received from beacon
    gpio_pad_select_gpio(GREENLED);
    gpio_set_direction(GREENLED, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(YELLOWLED);
    gpio_set_direction(YELLOWLED, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(REDLED);
    gpio_set_direction(REDLED, GPIO_MODE_OUTPUT);
}

void recieve_task(){ //task to continuously be receiving for beacons
    uint8_t *data_in = (uint8_t *) malloc(BUF_SIZE);
    while (1) {
        int len_in = uart_read_bytes(UART_NUM_1, data_in, BUF_SIZE, 20 / portTICK_RATE_MS);
        if (len_in > 0) {
            for (int i = 0; i < 7; i++) { //iterates through data
                if (data_in[i] == 0x1B) { // if start byte found
                    color = data_in[i+1]; //next byte is color
                    id = (int)data_in[i+2]; //next byte is id
                }
            }
            printf("Received\n");
        } else {
            printf("Nothing received\n");
            color = 'O'; //set LEDs back to OFF
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    free(data_in);
}

void led_task(){
    // keep times of last signal received to filter
    time_t lastReceived = time(NULL);
    time_t lapStart = time(NULL);
    char lastID = id;

    // converting time for alpha display
    char sec_char[20];
    char min_char[20];

    while(1) {
        switch(color){ //light up LEDs and controlsbased on color from beacon
            case 'R' :
                gpio_set_level(YELLOWLED, 0);//set led
                gpio_set_level(REDLED, 1);
                gpio_set_level(GREENLED, 0);
                driveSpeed = 1400; //stop driving when red
                lastReceived = time(NULL); //record time for split

                if (lastID != id) { //if beacon id changed then record split time
                    time_t lapTime = difftime(time(NULL), lapStart); //get split time in seconds
                    lapStart = time(NULL); //restart time
                    lastID = id; //save last beacon id
                    // Split time calculation
                    lapTime = (int)lapTime;
                    min = lapTime / 60;
                    sec = lapTime % 60;
                    sprintf(sec_char, "%d", sec);
                    sprintf(min_char, "%d", min);
                    // Write to time to display
                    writeTimeToDisplay(sec_char, min_char);

                }
                break;
            case 'G' :
                gpio_set_level(YELLOWLED, 0);//set led
                gpio_set_level(REDLED, 0);
                gpio_set_level(GREENLED, 1);
                driveSpeed = 1200; //start driving when green
                lastReceived = time(NULL);//record time for split

                if (lastID != id) { //if beacon id changed then record split time
                    time_t lapTime = difftime(time(NULL), lapStart); //get split time in seconds
                    lapStart = time(NULL); //restart time
                    lastID = id; //save last beacon id
                    // Split time calculation
                    lapTime = (int)lapTime;
                    min = lapTime / 60;
                    sec = lapTime % 60;
                    sprintf(sec_char, "%d", sec);
                    sprintf(min_char, "%d", min);
                    // Write to time to display
                    writeTimeToDisplay(sec_char, min_char);
                }
                break;
            case 'Y' :
                gpio_set_level(YELLOWLED, 1);//set led
                gpio_set_level(REDLED, 0);
                gpio_set_level(GREENLED, 0);
                driveSpeed = 1300; //slow down when yellow
                lastReceived = time(NULL); //record time for split

                if (lastID != id) { //if beacon id changed then record split time
                    time_t lapTime = difftime(time(NULL), lapStart); //get split time in seconds
                    lapStart = time(NULL); //restart time
                    lastID = id; //save last beacon id
                    // Split time calculation
                    lapTime = (int)lapTime;
                    min = lapTime / 60;
                    sec = lapTime % 60;
                    sprintf(sec_char, "%d", sec);
                    sprintf(min_char, "%d", min);
                    // Write to time to display
                    writeTimeToDisplay(sec_char, min_char);
                }
                break;
            case 'O' :
                // only perform this if not receiving anything for over a second (filtering incoming values)
                if (difftime(time(NULL), lastReceived) > 1) {
                    gpio_set_level(YELLOWLED, 0); //set led
                    gpio_set_level(REDLED, 0);
                    gpio_set_level(GREENLED, 0);
                    driveSpeed = 1200; //keep driving
                }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////

void app_main() {
    //servo control
    printf("Testing servo motor.......\n");
    mcpwm_initialize();

    //Lidar
    i2c_master_init();
    i2c_scanner();
    
    ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0) ); 
    esp_vfs_dev_uart_use_driver(UART_NUM_0); 

    //alphanumeric init
    int ret;
    printf(">> Test Alphanumeric Display: \n");
    
    // Set up routines
    // Turn on alpha oscillator
    ret = alpha_oscillator();
    if(ret == ESP_OK) {printf("- oscillator: ok \n");}

    // Set display blink off
    ret = no_blink();
    if(ret == ESP_OK) {printf("- blink: off \n");}
    ret = set_brightness_max(0xF);
    if(ret == ESP_OK) {printf("- brightness: max \n");}

    //hub
    uart_init();
    led_init();

    //ir
    check_efuse();
    adc_config();

    //speed
    /* Initialize PCNT event queue and PCNT functions */
    pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
    pcnt_example_init();
    timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    example_tg0_timer_init(TIMER_1, TEST_WITH_RELOAD, TIMER_INTERVAL1_SEC);

    //UDP init
    ESP_ERROR_CHECK(nvs_flash_init());
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());

    // Initialize display to 0000
    char minStr[20];
    char secStr[20];
    sprintf(minStr, "0");
    sprintf(secStr, "0");
    writeTimeToDisplay(secStr, minStr);

    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL); //task for sockets and wifi
    xTaskCreate(recieve_task, "uart_rx_task", 1024*4, NULL, configMAX_PRIORITIES, &task_rec); //task to receive 
    xTaskCreate(led_task, "set_gpio_task", 1024*2, NULL, configMAX_PRIORITIES-3, &task_led); //task for lighting up leds and driving control
    xTaskCreate(collision_task, "collision_task", 4096, NULL, 5, &task_coll); //task for collision prevention and turning
    xTaskCreate(ir_task, "ir_task", 4096, NULL, 5, &task_ir); //task to get sensor values
    xTaskCreate(speed_task, "speed_task", 2048, NULL, 5, &task_speed); //task for reading speed values

    bool justSwitched = 1;
    while(1){
        if (isAuto == 0) {
            //printf("manual\n");
            vTaskSuspend(task_rec);
            vTaskSuspend(task_led);
            vTaskSuspend(task_coll);
            vTaskSuspend(task_ir);
            vTaskSuspend(task_speed);
            if (justSwitched) {
                manualDrive = 1400;
                manualTurn = 1400;
                justSwitched = 0;
                motorTurn(manualTurn);
                motorDrive(manualDrive);
            }
        } else if (isAuto == 1) {
            //printf("autonomous\n");
            vTaskResume(task_rec);
            vTaskResume(task_led);
            vTaskResume(task_coll);
            vTaskResume(task_ir);
            vTaskSuspend(task_speed);
            justSwitched = 1;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
