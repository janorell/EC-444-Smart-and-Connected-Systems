#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/param.h>
#include <time.h>
#include <math.h>
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
//#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "./ADXL343.h"

/* The examples use simple WiFi configuration that you can set via
   'make menuconfig'.
   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_WIFI_SSID "Group_7"
#define EXAMPLE_WIFI_PASS "smart444"
// #define HOST_IP_ADDR "192.168.1.136"
#define HOST_IP_ADDR "192.168.1.124"
// maybe switch to other IP (bc destAddr)
#define PORT 8081

#define GPIO_WET_LED     4

#define DEFAULT_VREF    1100
#define NO_OF_SAMPLES   64

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
#define SLAVE_ADDR                         ADXL343_ADDRESS // 0x53

static esp_adc_cal_characteristics_t * adc_chars;
static const adc_channel_t battery_channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_channel_t therm_channel = ADC_CHANNEL_3;
static const adc_atten_t atten = ADC_ATTEN_DB_11; // attenuation - db 11 (for full range)
static const adc_unit_t unit = ADC_UNIT_1;

static void check_efuse(){
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    }
    else {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    }
    else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type){
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    }
    else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    }
    else {
        printf("Characterized using Default Vref\n");
    }
}

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

// Utility  Functions //////////////////////////////////////////////////////////

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

// ADXL343 Functions ///////////////////////////////////////////////////////////

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
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, &data1, ACK_VAL);
  i2c_master_read_byte(cmd, &data2, ACK_CHECK_DIS);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  int16_t data2_16 = data2 << 8;
  data3 = data2_16 | data1;
  return data3;
}

void setRange(range_t range) {
  /* Red the data format register to preserve bits */
  uint8_t format = readRegister(ADXL343_REG_DATA_FORMAT);

  /* Update the data rate */
  format &= ~0x0F;
  format |= range;

  /* Make sure that the FULL-RES bit is enabled for range scaling */
  format |= 0x08;

  /* Write the register back to the IC */
  writeRegister(ADXL343_REG_DATA_FORMAT, format);
}

range_t getRange(void) {
  /* Red the data format register to preserve bits */
  return (range_t)(readRegister(ADXL343_REG_DATA_FORMAT) & 0x03);
}

dataRate_t getDataRate(void) {
  return (dataRate_t)(readRegister(ADXL343_REG_BW_RATE) & 0x0F);
}

////////////////////////////////////////////////////////////////////////////////
float pitch;

// function to get acceleration
void getAccel(float * xp, float *yp, float *zp) {
  *xp = read16(ADXL343_REG_DATAX0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  *yp = read16(ADXL343_REG_DATAY0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  *zp = read16(ADXL343_REG_DATAZ0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
}

// function to print roll and pitch and return pitch
float calcRP(float x, float y, float z){
  pitch = atan2((-x) , sqrt(y * y + z * z)) * 57.3;
  return pitch;
}

int countStep(int steps, bool upswing, float upswingThreshold, float downswingThreshold){
  float xVal, yVal, zVal;
  getAccel(&xVal, &yVal, &zVal);
  pitch = calcRP(xVal, yVal, zVal);

  if(upswing) {
    if(pitch < upswingThreshold) {
      steps++;
      return steps;
    }
  } else {
    if(pitch > downswingThreshold) {
      steps++;
      return steps;
    }
  }
  return 0;
  vTaskDelay(100 / portTICK_RATE_MS);
}

float thermistor() {
  //char output[100];
        //while (1) {
            //double volt;
  uint32_t adc_reading_TH = 0;
  //uint32_t adc_reading = 0;
  for (int i = 0; i < NO_OF_SAMPLES; i++) {
    if (unit == ADC_UNIT_1) {
      adc_reading_TH += adc1_get_raw((adc1_channel_t)therm_channel);
    } else {
      int raw;
      adc2_get_raw((adc2_channel_t)therm_channel, ADC_WIDTH_BIT_12, &raw);
      adc_reading_TH += raw;
    }
  }
  adc_reading_TH /= NO_OF_SAMPLES;
  uint32_t voltage_TH = esp_adc_cal_raw_to_voltage(adc_reading_TH, adc_chars);
  double volt_TH = (double)voltage_TH / 1000;
  // THERMISTOR
  double Rx;
  Rx = (double)(50000 - (10000 * volt_TH)) / volt_TH;
  double y;
  y = (double)298.15 * log(10000 / Rx);
  double temp;
  temp = (double)(85875 + (y * 273.15)) / (3435 - y);

            //sprintf(output, "temp:%f\n", temp);
            //printf("%s", output);
            //vTaskDelay(100 / portTICK_RATE_MS);
  return temp;
        //}
}

float batterymonitor(){
  //char output[100];
            //while (1) {
                //double volt;
  uint32_t adc_reading_B = 0;
                //uint32_t adc_reading = 0;
  for (int i = 0; i < NO_OF_SAMPLES; i++) {
    if (unit == ADC_UNIT_1) {
      adc_reading_B += adc1_get_raw((adc1_channel_t)battery_channel);
    } else {
      int raw;
      adc2_get_raw((adc2_channel_t)battery_channel, ADC_WIDTH_BIT_12, &raw);
      adc_reading_B += raw;
    }
  }
  adc_reading_B /= NO_OF_SAMPLES;
  uint32_t voltage_B = esp_adc_cal_raw_to_voltage(adc_reading_B, adc_chars);
  float voltdivide;
                       //printf("Raw: %d\tVoltage: %dmV\n", adc_reading_B, voltage_B);
  voltdivide = (((float)voltage_B/1000.00)*2000.00)/(1000.00);
                      // printf("Vout: %fV\n", voltdivide)

                //sprintf(output, "Battery:%f\n", voltdivide);
                //printf("%s", output);
                //vTaskDelay(100 / portTICK_RATE_MS);
  return voltdivide;
            //}
}
/*
#ifdef CONFIG_EXAMPLE_IPV4
#define HOST_IP_ADDR 192.168.1.11
#else
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#endif
*/

// #define PORT CONFIG_EXAMPLE_PORT

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

const int IPV4_GOTIP_BIT = BIT0;
const int IPV6_GOTIP_BIT = BIT1;

static const char *TAG = "example";
// used for printf- essentially console.log
//char *payload = "Message from ESP32 ";

static esp_err_t event_handler(void *ctx, system_event_t *event){
  switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_START");
      break;
    case SYSTEM_EVENT_STA_CONNECTED:
        /* enable ipv6 */
        tcpip_adapter_create_ip6_linklocal(TCPIP_ADAPTER_IF_STA);
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, IPV4_GOTIP_BIT);
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP");
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, IPV4_GOTIP_BIT);
        xEventGroupClearBits(wifi_event_group, IPV6_GOTIP_BIT);
        break;
    case SYSTEM_EVENT_AP_STA_GOT_IP6:
        xEventGroupSetBits(wifi_event_group, IPV6_GOTIP_BIT);
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP6");

        char *ip6 = ip6addr_ntoa(&event->event_info.got_ip6.ip6_info.ip);
        ESP_LOGI(TAG, "IPv6: %s", ip6);
    default:
        break;
    }
    return ESP_OK;
}

static void initialise_wifi(void){
  tcpip_adapter_init();
  wifi_event_group = xEventGroupCreate();
  ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL));
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
  ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM));
  wifi_config_t wifi_config = {
    .sta = {
        .ssid = EXAMPLE_WIFI_SSID,
        .password = EXAMPLE_WIFI_PASS,
    },
  };
  ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
  ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
  ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
  ESP_ERROR_CHECK( esp_wifi_start() );
}

static void wait_for_ip(){
  uint32_t bits = IPV4_GOTIP_BIT | IPV6_GOTIP_BIT ;

  ESP_LOGI(TAG, "Waiting for AP connection...");
  xEventGroupWaitBits(wifi_event_group, bits, false, true, portMAX_DELAY);
  ESP_LOGI(TAG, "Connected to AP");
}

/*static void udp_client_task(void *pvParameters){
  char rx_buffer[128];
  char addr_str[128];
  int addr_family;
  int ip_protocol;
  while (1) {
    #ifdef CONFIG_EXAMPLE_IPV4
      struct sockaddr_in destAddr;
      destAddr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
      destAddr.sin_family = AF_INET;
      destAddr.sin_port = htons(PORT);
      addr_family = AF_INET;
      ip_protocol = IPPROTO_IP;
      inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
    #else // IPV6
      struct sockaddr_in6 destAddr;
      inet6_aton(HOST_IP_ADDR, &destAddr.sin6_addr);
      destAddr.sin6_family = AF_INET6;
      destAddr.sin6_port = htons(PORT);
      addr_family = AF_INET6;
      ip_protocol = IPPROTO_IPV6;
      inet6_ntoa_r(destAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
    #endif

    int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
      ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
      break;
    }
    ESP_LOGI(TAG, "Socket created");
    char payload[200];
    while (1) {
      double temp = 1.0;
      double steps = 2.5;
      double vol = 3.5;
      sprintf(payload, "{\"temp\":\"%f\",\"steps\":\"%f\",\"voltage\":\"%f\"}\n", temp, steps, vol);
      int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&destAddr, sizeof(destAddr));
      if (err < 0) {
        ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
        break;
      }
      ESP_LOGI(TAG, "Message sent");

      struct sockaddr_in sourceAddr; // Large enough for both IPv4 or IPv6
      socklen_t socklen = sizeof(sourceAddr);
      int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&sourceAddr, &socklen);

      // Error occured during receiving
      if (len < 0) {
        ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
        break;
      } else { // Data received
        rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
        ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
        ESP_LOGI(TAG, "%s", rx_buffer);
      }
      vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    if (sock != -1) {
      ESP_LOGE(TAG, "Shutting down socket and restarting...");
        shutdown(sock, 0);
        close(sock);
    }
  }
  vTaskDelete(NULL);
}*/

void app_main(){
  ESP_ERROR_CHECK( nvs_flash_init() );
  initialise_wifi();
  wait_for_ip();
  check_efuse();

  if (unit == ADC_UNIT_1) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(battery_channel, atten);
    adc1_config_channel_atten(therm_channel, atten);
  } else {
    adc2_config_channel_atten((adc2_channel_t)battery_channel, atten);
    adc2_config_channel_atten((adc2_channel_t)therm_channel, atten);
  }

  adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
  print_char_val_type(val_type);

  //check_efuse();

// if (unit == ADC_UNIT_1) {
//     adc1_config_width(ADC_WIDTH_BIT_12);
//     //adc1_config_channel_atten(therm_channel, atten);
// } else {
//     adc2_config_channel_atten((adc2_channel_t)therm_channel, atten);
// }

// adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
// esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
// print_char_val_type(val_type);

  gpio_pad_select_gpio(GPIO_WET_LED);
  gpio_set_direction(GPIO_WET_LED, GPIO_MODE_OUTPUT);

  // Routine
  i2c_master_init();
  i2c_scanner();

  uint8_t deviceID;
  getDeviceID(&deviceID);
  if (deviceID == 0xE5) {
    printf("\n>> Found ADAXL343\n");
  }

  // Disable interrupts
  writeRegister(ADXL343_REG_INT_ENABLE, 0);

  // Set range
  setRange(ADXL343_RANGE_16_G);

  // Display range
  printf  ("- Range:         +/- ");
  switch(getRange()) {
    case ADXL343_RANGE_16_G:
      printf  ("16 ");
      break;
    case ADXL343_RANGE_8_G:
      printf  ("8 ");
      break;
    case ADXL343_RANGE_4_G:
      printf  ("4 ");
      break;
    case ADXL343_RANGE_2_G:
      printf  ("2 ");
      break;
    default:
      printf  ("?? ");
      break;
  }
  printf(" g\n");

  // Display data rate
  printf ("- Data Rate:    ");
  switch(getDataRate()) {
    case ADXL343_DATARATE_3200_HZ:
      printf  ("3200 ");
      break;
    case ADXL343_DATARATE_1600_HZ:
      printf  ("1600 ");
      break;
    case ADXL343_DATARATE_800_HZ:
      printf  ("800 ");
      break;
    case ADXL343_DATARATE_400_HZ:
      printf  ("400 ");
      break;
    case ADXL343_DATARATE_200_HZ:
      printf  ("200 ");
      break;
    case ADXL343_DATARATE_100_HZ:
      printf  ("100 ");
      break;
    case ADXL343_DATARATE_50_HZ:
      printf  ("50 ");
      break;
    case ADXL343_DATARATE_25_HZ:
      printf  ("25 ");
      break;
    case ADXL343_DATARATE_12_5_HZ:
      printf  ("12.5 ");
      break;
    case ADXL343_DATARATE_6_25HZ:
      printf  ("6.25 ");
      break;
    case ADXL343_DATARATE_3_13_HZ:
      printf  ("3.13 ");
      break;
    case ADXL343_DATARATE_1_56_HZ:
      printf  ("1.56 ");
      break;
    case ADXL343_DATARATE_0_78_HZ:
      printf  ("0.78 ");
      break;
    case ADXL343_DATARATE_0_39_HZ:
      printf  ("0.39 ");
      break;
    case ADXL343_DATARATE_0_20_HZ:
      printf  ("0.20 ");
      break;
    case ADXL343_DATARATE_0_10_HZ:
      printf  ("0.10 ");
      break;
    default:
      printf  ("???? ");
      break;
  }
  printf(" Hz\n\n");

  // Enable measurements
  writeRegister(ADXL343_REG_POWER_CTL, 0x08);

  // xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
  char rx_buffer[128];
  char addr_str[128];
  int addr_family;
  int ip_protocol;
  while (1) {
    #ifdef CONFIG_EXAMPLE_IPV4
      struct sockaddr_in destAddr;
      destAddr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
      destAddr.sin_family = AF_INET;
      destAddr.sin_port = htons(PORT);
      addr_family = AF_INET;
      ip_protocol = IPPROTO_IP;
      inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
    #else // IPV6
      struct sockaddr_in6 destAddr;
      inet6_aton(HOST_IP_ADDR, &destAddr.sin6_addr);
      destAddr.sin6_family = AF_INET6;
      destAddr.sin6_port = htons(PORT);
      addr_family = AF_INET6;
      ip_protocol = IPPROTO_IPV6;
      inet6_ntoa_r(destAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
    #endif

    int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
      ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
      break;
    }

    ESP_LOGI(TAG, "Socket created");
      char payload[200];
      int steps = 0;
      bool upswing = true;
      float upswingThreshold = -40;
      float downswingThreshold = 15;
      int newStep;

      float temp;
      float battery;

      time_t time_2, time_3;
      time_2 = time(NULL);
      time_3 = time(NULL);

        while (1) {
          //while (1) {
            //wet LED
            if(difftime(time(NULL), time_2) > 5.0){
              time_2 = time(NULL);
              gpio_set_level(GPIO_WET_LED, 1);
              vTaskDelay(500 / portTICK_RATE_MS);
              gpio_set_level(GPIO_WET_LED, 0);
            }

            temp = thermistor();
            battery = batterymonitor();
            //step
            newStep = countStep(steps, upswing, upswingThreshold, downswingThreshold);
            if (newStep != 0) {
              ESP_LOGI(TAG, " current step: %d, temp: %f, battery: %f\n", newStep, temp, battery);
              steps = newStep;
              upswing = !upswing;
              sprintf(payload, "{\"steps\":\"%d\",\"temp\":\"%f\",\"voltage\":\"%f\"}\n", steps, temp, battery);
              int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&destAddr, sizeof(destAddr));
              if (err < 0) {
                ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
                break;
              }
              ESP_LOGI(TAG, "Message sent: %s", payload);

              struct sockaddr_in sourceAddr; // Large enough for both IPv4 or IPv6
              socklen_t socklen = sizeof(sourceAddr);
              int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&sourceAddr, &socklen);

              // Error occured during receiving
              if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
              } else { // Data received
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);
              }
            } else {
              if(difftime(time(NULL), time_3) > 2.0){
                time_3 = time(NULL);
                ESP_LOGI(TAG, " current step: %d, temp: %f, battery: %f\n", steps, temp, battery);
                sprintf(payload, "{\"steps\":\"%d\",\"temp\":\"%f\",\"voltage\":\"%f\"}\n", steps, temp, battery);
                int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&destAddr, sizeof(destAddr));
                if (err < 0) {
                  ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
                  break;
                }
                ESP_LOGI(TAG, "Message sent: %s", payload);

                struct sockaddr_in sourceAddr; // Large enough for both IPv4 or IPv6
                socklen_t socklen = sizeof(sourceAddr);
                int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&sourceAddr, &socklen);

                // Error occured during receiving
                if (len < 0) {
                  ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                  break;
                } else { // Data received
                  rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                  ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                  ESP_LOGI(TAG, "%s", rx_buffer);
                }
              }
            }
            // int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&destAddr, sizeof(destAddr));
            // if (err < 0) {
            //   ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
            //   break;
            // }
            // ESP_LOGI(TAG, "Message sent: %s", payload);

            // struct sockaddr_in sourceAddr; // Large enough for both IPv4 or IPv6
            // socklen_t socklen = sizeof(sourceAddr);
            // int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&sourceAddr, &socklen);

            // // Error occured during receiving
            // if (len < 0) {
            //   ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            //   break;
            // } else { // Data received
            //   rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
            //   ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
            //   ESP_LOGI(TAG, "%s", rx_buffer);
            // }
        }
        if (sock != -1) {
          ESP_LOGE(TAG, "Shutting down socket and restarting...");
          shutdown(sock, 0);
          close(sock);
        }
  }
}
