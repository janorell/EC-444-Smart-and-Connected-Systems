/*
  - Quest5 
  - hub code
  Team7: Vanessa Schuweh, Vindhya Kuchibhotla, Jennifer Norell
  November 21, 2019
*/

#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "soc/rmt_reg.h"
#include "driver/uart.h"
#include "driver/periph_ctrl.h"
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
#define BLUELED   14
#define GREENLED  32
#define REDLED    15

//wifi config
#ifdef CONFIG_EXAMPLE_IPV4
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#else
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#endif
#define PORT CONFIG_EXAMPLE_PORT

char start = 0x0A;
char id = 'O';
char code = 'U';
int hubID = 7;

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
    while (1) {
      sprintf(payload, "{\"hubID\":\"%d\",\"fobID\":\"%c\",\"code\":\"%c\"}\n", hubID, id, code); //sending in JSON
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
  gpio_pad_select_gpio(BLUELED);
  gpio_set_direction(BLUELED, GPIO_MODE_OUTPUT);
  gpio_pad_select_gpio(GREENLED);
  gpio_set_direction(GREENLED, GPIO_MODE_OUTPUT);
  gpio_pad_select_gpio(REDLED);
  gpio_set_direction(REDLED, GPIO_MODE_OUTPUT);
}

void recieve_task(){
  uint8_t *data_in = (uint8_t *) malloc(BUF_SIZE);
  while (1) {
    int len_in = uart_read_bytes(UART_NUM_1, data_in, BUF_SIZE, 20 / portTICK_RATE_MS);
    if (len_in > 0) {
      for (int i = 0; i < 20; i++) { //iterates through data
        if (data_in[i] == 0x0A) { // if start byte found
          id = data_in[i+1]; //next byte is id
          code = data_in[i+2]; //next byte is code
        }
      }
      printf("Received\n");
    } else{
      printf("Nothing received\n");
      id = 'O'; //set LEDs back to OFF
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  free(data_in);
}

void led_task(){
  while(1) {
    switch(id){ //light up LEDs based on FOB ID
      case '1' :
        gpio_set_level(GREENLED, 0);
        gpio_set_level(REDLED, 1);
        gpio_set_level(BLUELED, 0);
        break;
      case '3' :
        gpio_set_level(GREENLED, 0);
        gpio_set_level(REDLED, 0);
        gpio_set_level(BLUELED, 1);
        break;
      case '2' :
        gpio_set_level(GREENLED, 1);
        gpio_set_level(REDLED, 0);
        gpio_set_level(BLUELED, 0);
        break;
      case 'O' :
        gpio_set_level(GREENLED, 0);
        gpio_set_level(REDLED, 0);
        gpio_set_level(BLUELED, 0);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void app_main() {
  ESP_ERROR_CHECK(nvs_flash_init());
  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK(example_connect());

  uart_init();
  led_init();

  xTaskCreate(recieve_task, "uart_rx_task", 1024*4, NULL, configMAX_PRIORITIES, NULL);
  xTaskCreate(led_task, "set_gpio_task", 1024*2, NULL, configMAX_PRIORITIES-3, NULL);
  xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
}