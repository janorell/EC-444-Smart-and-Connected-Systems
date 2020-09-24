/*
  - Quest5 
  - fob 2 code
  Team7: Vanessa Schuweh, Vindhya Kuchibhotla, Jennifer Norell
  November 21, 2019
*/

#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/rmt.h"
#include "soc/rmt_reg.h"
#include "driver/uart.h"
#include "driver/periph_ctrl.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "protocol_examples_common.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

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
#define BLUEPIN   14
#define GREENPIN  32
#define REDPIN    15

#ifdef CONFIG_EXAMPLE_IPV4
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#else
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#endif

#define PORT CONFIG_EXAMPLE_PORT

static const char *TAG = "example";
static const char *payload = "Message from ESP32 ";

char start = 0x0A;
int length_out = 3;
char status = 'O';
char id = 'O';
char code = 'O';
int flag  = 0;

static void udp_client_task(void *pvParameters){
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

    while (1) {
      int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
      if (err < 0) {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        break;
      }
      ESP_LOGI(TAG, "Message sent %s", payload);

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
        if (strcmp(rx_buffer, "1") == 0) { // if received 1, light up green LED
          gpio_set_level(GREENPIN, 1);
          vTaskDelay(2000 / portTICK_PERIOD_MS);
          gpio_set_level(GREENPIN, 0);
        }
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
}

static xQueueHandle gpio_evt_queue = NULL;

// Button interrupt handler -- add to queue
static void IRAM_ATTR gpio_isr_handler(void* arg){
  uint32_t gpio_num = (uint32_t) arg;
  xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// RMT tx init
static void rmt_tx_init() {
    rmt_config_t rmt_tx;
    rmt_tx.channel = RMT_TX_CHANNEL;
    rmt_tx.gpio_num = RMT_TX_GPIO_NUM;
    rmt_tx.mem_block_num = 1;
    rmt_tx.clk_div = RMT_CLK_DIV;
    rmt_tx.tx_config.loop_en = false;
    rmt_tx.tx_config.carrier_duty_percent = 50;
    rmt_tx.tx_config.carrier_freq_hz = 38000;
    rmt_tx.tx_config.carrier_level = 1;
    rmt_tx.tx_config.carrier_en = 1;
    rmt_tx.tx_config.idle_level = 1;
    rmt_tx.tx_config.idle_output_en = true;
    rmt_tx.rmt_mode = 0;
    rmt_config(&rmt_tx);
    rmt_driver_install(rmt_tx.channel, 0, 0);
}

static void uart_init() {
  uart_config_t uart_config = {
      .baud_rate = 2400,
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
    gpio_pad_select_gpio(BLUEPIN);
    gpio_pad_select_gpio(GREENPIN);
    gpio_pad_select_gpio(REDPIN);
    gpio_set_direction(BLUEPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(GREENPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(REDPIN, GPIO_MODE_OUTPUT);
}

static void hw_int_init() {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    gpio_intr_enable(GPIO_INPUT_IO_1 );
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
}

void hardware_task(){
  uint32_t io_num;
  while(1) {
    if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
      flag = 1; //if button press, set flag high
    }
    vTaskDelay(1000/ portTICK_PERIOD_MS);
    flag = 0;
  }

}

void transmit_task(){
  char *data_out = (char *) malloc(length_out);
  while(1) {
    if (flag == 1){ // if button press send id and code
      id = '2';
      code = 'K';
      status = 'B';
      printf("Authorizing\n");
    } else { // else default id and code (won't authorize)
      id = 'O';
      code = 'U';
      status = 'O';
    }
    data_out[0] = start;
    data_out[1] = id;
    data_out[2] = code;
    uart_write_bytes(UART_NUM_1, data_out, length_out+1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void led_task(){
  while(1) {
    switch(status){
      case 'B' :
        gpio_set_level(GREENPIN, 0);
        gpio_set_level(BLUEPIN, 1);
        break;
      case 'G' :
        gpio_set_level(GREENPIN, 1);
        gpio_set_level(BLUEPIN, 0);
        break;
      case 'O' :
        gpio_set_level(GREENPIN, 0);
        gpio_set_level(BLUEPIN, 0);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void app_main() {
  ESP_ERROR_CHECK(nvs_flash_init());
  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK(example_connect());

  rmt_tx_init();
  uart_init();
  led_init();
  hw_int_init();

  xTaskCreate(transmit_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
  xTaskCreate(led_task, "set_gpio_task", 1024*2, NULL, configMAX_PRIORITIES-3, NULL);
  xTaskCreate(hardware_task, "hardware_task", 1024*2, NULL, configMAX_PRIORITIES-3, NULL);
  xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
}