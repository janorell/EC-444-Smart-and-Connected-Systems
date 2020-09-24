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

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_adc_cal.h"
#include "esp_vfs_dev.h"

#include "driver/mcpwm.h"
#include "driver/i2c.h"
#include "driver/rmt.h"
#include "driver/uart.h"
#include "driver/periph_ctrl.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/timer.h"

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

int distCollision; //Lidar
//bool driveOK = 1; //RYAN
int driveSpeed = 1200;
double distance_IR1;
double distance_IR2;
double distance_IR3;

unsigned int secCount = 9;
unsigned int minCount = 10;

int isAuto = -1;

xTaskHandle task_rec;
xTaskHandle task_led;
xTaskHandle task_ir;
xTaskHandle task_coll;

static void recieve_task ();
static void led_task ();
static void collision_task ();
static void ir_task ();

int manualDrive = 1400;
int manualTurn = 1400;

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

void motorDrive(int speedVal) {
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, speedVal);
}

void motorTurn(int turnVal){
  mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, turnVal);
}

//////////////////////////////////////////////////////////////////////////////////////////////////

// Lidar /////////////////////////////////////////////////////////////////////////////////////////
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

void ir_task(){
  while (1) {
		double volt_IR1;
    double volt_IR2;
    double volt_IR3;
    uint32_t adc_reading_IR1 = 0;
    uint32_t adc_reading_IR2 = 0;
    uint32_t adc_reading_IR3 = 0;
		int ret;

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

		if (volt_IR1 > 1) {
			distance_IR1 = (double)(volt_IR1 - 3.316) / -0.0431;
		} else if (1 > volt_IR1 || volt_IR1 < 0.5) {
			distance_IR1 = (double)(volt_IR1 - 1.5211) / -0.0099;
		} else {
			distance_IR1 = (double)(volt_IR1 - 1.3335) / -0.0078;
		}
		distance_IR1 = (double)distance_IR1 / 100;
    printf("Side IR Distance 1: %f\n", distance_IR1);

    if (volt_IR2 > 1) {
			distance_IR2 = (double)(volt_IR2 - 3.316) / -0.0431;
		} else if (1 > volt_IR2 || volt_IR2 < 0.5) {
			distance_IR2 = (double)(volt_IR2 - 1.5211) / -0.0099;
		} else {
			distance_IR2 = (double)(volt_IR2 - 1.3335) / -0.0078;
		}
		distance_IR2 = (double)distance_IR2 / 100;
    printf("Side IR Distance 2: %f\n", distance_IR2);

    if (volt_IR3 > 1) {
			distance_IR3 = (double)(volt_IR3 - 3.316) / -0.0431;
		} else if (1 > volt_IR3 || volt_IR3 < 0.5) {
			distance_IR3 = (double)(volt_IR3 - 1.5211) / -0.0099;
		} else {
			distance_IR3 = (double)(volt_IR3 - 1.3335) / -0.0078;
		}
		distance_IR3 = (double)distance_IR3 / 100;
    printf("Back IR Distance 3: %f\n", distance_IR3);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////

// Lidar /////////////////////////////////////////////////////////////////////////////////////////
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

// ADXL343
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

int lidarGetRange() {
  int val = 0;

  writeRegister(RegisterMeasure, MeasureValue);
  vTaskDelay(200 / portTICK_RATE_MS);
  val = read16(RegisterHighLowB);

  printf("Distance: %d\n", val);
  return val;
}

void collision_task(){
	while(1){
		distCollision = lidarGetRange();
		if (distCollision <= 100) { // stop and adjust
			printf("Stopping and adjusting\n");

      motorDrive(1400); //stop
      vTaskDelay(500 / portTICK_PERIOD_MS);
      // motorTurn(700); //hard right
      // vTaskDelay(750 / portTICK_PERIOD_MS);

      // backwards until too close to back wall
      while (distance_IR3 >= 150) {
        motorTurn(700); //hard right
        vTaskDelay(500 / portTICK_PERIOD_MS);
        motorDrive(1550);
      }
      // motorDrive(1500);//slow back for a second
      // vTaskDelay(1000 / portTICK_PERIOD_MS);
      motorDrive(1400); //stop
      motorTurn(2300); //turn left
      vTaskDelay(500 / portTICK_PERIOD_MS);


      // driveSpeed = 1400;
      // motorDrive(driveSpeed); //stop
      // vTaskDelay(1000 / portTICK_PERIOD_MS);
      // motorTurn(700); //hard right
      // vTaskDelay(1000 / portTICK_PERIOD_MS);
      // motorDrive(1600);//slow back for half a second
      // vTaskDelay(500 / portTICK_PERIOD_MS);
      // motorDrive(1400); //stop
      // vTaskDelay(1000 / portTICK_PERIOD_MS);
      // motorTurn(2300);//hard left
      // vTaskDelay(1000 / portTICK_PERIOD_MS);
      // motorDrive(1200);//slow forward
      // vTaskDelay(1000 / portTICK_PERIOD_MS);
      //perform until sensors are even and disCollision is farther than say 100cm

			// motorDrive(1400); //stop
			//motorDrive();
			//motorBackward();
		} else if (distCollision <= 250) { // start turning
      printf("Starting left turn \n");
      motorTurn(2300); // hard left
      // motorDrive((int)(driveSpeed + (1400 - driveSpeed)/2)); // slight slowdown
      motorDrive(driveSpeed);

    } else {
      // keep 'er straight
      // setting buffer of 50cm-150cm from wall
      int turnRadius = 1400;
      if (distance_IR1 >= 1.5) { // too far from wall
      	printf("Too far from wall, slight right\n");
        turnRadius = 1100;
      } else if (distance_IR1 <= .25) { // too close to wall
        printf("Too close to wall, turning left\n");
        turnRadius = 1700;
      } else if (distance_IR1 - distance_IR2 >= .1){ // turn right to straighten
        printf("Straightening right\n");
        turnRadius = 1100;
      } else if (distance_IR1 - distance_IR2 <= -.1){ // turn left to straighten
        printf("Straightening left\n");
        turnRadius = 1700;
      }

      motorTurn(turnRadius);
			printf("Drive\n");
    	motorDrive(driveSpeed);



			// motorDrive(1200); //forward slow
			//motorStop();
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
      sprintf(split, "%u:%u", minCount, secCount);
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
          ESP_LOGI(TAG, "Fast");
        //set global variable
          if (manualDrive <= 700){
            manualDrive = 700;
          } else if (manualDrive > 1300){
            manualDrive = 1300;
          } else {
            manualDrive -= 50;
          }
        } else if (strcmp(rx_buffer, "2") == 0) {
          ESP_LOGI(TAG, "Slow");
          //set global variable
          if (manualDrive >= 1300 ){
            manualDrive = 1400;
          } else {
            manualDrive += 50;
          }
        }
        else if (strcmp(rx_buffer, "3") == 0) {
          ESP_LOGI(TAG, "Left");
          //set global variable
          if (manualTurn >= 2300 ){
            manualTurn = 2300;
          } else {
            manualTurn += 450;
          }
        }
        else if (strcmp(rx_buffer, "4") == 0) {
          ESP_LOGI(TAG, "Right");
          //set global variable
          if (manualTurn <= 700 ){
            manualTurn = 700;
          } else {
            manualTurn -= 450;
          }
        }
        else if (strcmp(rx_buffer, "-5") == 0) {
          ESP_LOGI(TAG, "Auto");
          isAuto = 1;
        }
        else if (strcmp(rx_buffer, "5") == 0) {
          ESP_LOGI(TAG, "Manual");
          isAuto = 0;
        }
      }
      printf("\n\nManual Drive: %d\nManual Turn: %d\n\n", manualDrive, manualTurn);
      motorDrive(manualDrive);
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

static xQueueHandle gpio_evt_queue = NULL;

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

static void led_init() {
  gpio_pad_select_gpio(GREENLED);
  gpio_set_direction(GREENLED, GPIO_MODE_OUTPUT);
  gpio_pad_select_gpio(YELLOWLED);
  gpio_set_direction(YELLOWLED, GPIO_MODE_OUTPUT);
  gpio_pad_select_gpio(REDLED);
  gpio_set_direction(REDLED, GPIO_MODE_OUTPUT);
}

char genCheckSum(char *p, int len) {
  char temp = 0;
  for (int i = 0; i < len; i++){
    temp = temp^p[i];
  }
  // printf("%X\n",temp);

  return temp;
}

void recieve_task(){
  uint8_t *data_in = (uint8_t *) malloc(BUF_SIZE);
  while (1) {
    int len_in = uart_read_bytes(UART_NUM_1, data_in, BUF_SIZE, 20 / portTICK_RATE_MS);
    if (len_in > 0) {
      for (int i = 0; i < 20; i++) { //iterates through data
        if (data_in[i] == 0x1B) { // if start byte found
          color = data_in[i+1]; //next byte is color
          printf("color: %c\n", color);
          id = (int)data_in[i+2]; //next byte is id
          printf("id: %d\n",id);
          checksum = data_in[1+3]; //genCheckSum(data_out,len_out-1)
        }
      }
      //printf("data_in: %s\n", data_in);
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
  while(1) {
    switch(color){ //light up LEDs based on FOB ID
      case 'R' :
        gpio_set_level(YELLOWLED, 0);
        gpio_set_level(REDLED, 1);
        gpio_set_level(GREENLED, 0);
        driveSpeed = 1400;
        lastReceived = time(NULL);

        if (lastID != id) {
          time_t lapTime = difftime(time(NULL), lapStart);
          lapStart = time(NULL);
          lastID = id;
          printf("\n\n\nSplit: %d\n", (int)lapTime);
        }
		    // motorDrive(1400);
        break;
      case 'G' :
        gpio_set_level(YELLOWLED, 0);
        gpio_set_level(REDLED, 0);
        gpio_set_level(GREENLED, 1);
        driveSpeed = 1200;
        lastReceived = time(NULL);

        if (lastID != id) {
          time_t lapTime = difftime(time(NULL), lapStart);
          lapStart = time(NULL);
          lastID = id;
          printf("\n\n\nSplit: %d\n", (int)lapTime);
        }
		    // motorDrive(900);
        break;
      case 'Y' :
        gpio_set_level(YELLOWLED, 1);
        gpio_set_level(REDLED, 0);
        gpio_set_level(GREENLED, 0);
        driveSpeed = 1300;
        lastReceived = time(NULL);

        if (lastID != id) {
          time_t lapTime = difftime(time(NULL), lapStart);
          lapStart = time(NULL);
          lastID = id;
          printf("\n\n\nSplit: %d\n", (int)lapTime);
        }
		    // motorDrive(1200);
        break;
      case 'O' :
        // only perform this if not receiving anything for over a second
        if (difftime(time(NULL), lastReceived) > 1) {
          gpio_set_level(YELLOWLED, 0);
          gpio_set_level(REDLED, 0);
          gpio_set_level(GREENLED, 0);
          driveSpeed = 1200;
        };
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

	//hub
	uart_init();
  led_init();

  //ir
  check_efuse();
  adc_config();

  //UDP init
  ESP_ERROR_CHECK(nvs_flash_init());
  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK(example_connect());

  xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL); //task to receive message from web server
  xTaskCreate(recieve_task, "uart_rx_task", 1024*4, NULL, configMAX_PRIORITIES, &task_rec);
  xTaskCreate(led_task, "set_gpio_task", 1024*2, NULL, configMAX_PRIORITIES-3, &task_led);
  xTaskCreate(collision_task, "collision_task", 4096, NULL, 5, &task_coll);
  xTaskCreate(ir_task, "ir_task", 4096, NULL, 5, &task_ir);

  bool justSwitched = 1;
  while(1){
    if (isAuto == 0) {
      //printf("manual\n");
      vTaskSuspend(task_rec);
      vTaskSuspend(task_led);
      vTaskSuspend(task_coll);
      vTaskSuspend(task_ir);
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
      justSwitched = 1;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
