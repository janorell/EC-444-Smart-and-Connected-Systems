#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt.h"
#include "soc/rmt_reg.h"
#include "driver/uart.h"
#include "driver/periph_ctrl.h"


#define UART_TX_GPIO_NUM 26
#define UART_RX_GPIO_NUM 34
#define BUF_SIZE (1024)
#define rmt_item32_tIMEOUT_US   9500
#define GPIO_INPUT_IO_1       4
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INPUT_PIN_SEL    1ULL<<GPIO_INPUT_IO_1
#define RMT_TX_CHANNEL    1
#define RMT_TX_GPIO_NUM   25
#define RMT_CLK_DIV       100
#define RMT_TICK_10_US    (80000000/RMT_CLK_DIV/100000)
#define BLUE   14
#define GREEN  32
#define RED    15

char start = 0x00;
int length_out = 2;
char sstate = '1';
char check = '4';

static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg){
  uint32_t gpio_num = (uint32_t) arg;
  xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

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
void transmit(){
  char *data_out = (char *) malloc(length_out);
  while(1) {
    data_out[0] = start;
    data_out[1] = sstate;
    uart_write_bytes(UART_NUM_1, data_out, length_out+1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

static void led_init() {
  gpio_pad_select_gpio(BLUE);
  gpio_set_direction(BLUE, GPIO_MODE_OUTPUT);
  gpio_pad_select_gpio(GREEN);
  gpio_set_direction(GREEN, GPIO_MODE_OUTPUT);
  gpio_pad_select_gpio(RED);
  gpio_set_direction(RED, GPIO_MODE_OUTPUT);
}

void recieve() {
  uint8_t *data_in = (uint8_t *) malloc(BUF_SIZE);
  while (1) {
    int len_in = uart_read_bytes(UART_NUM_1, data_in, BUF_SIZE, 20 / portTICK_RATE_MS);
    if (len_in > 0) {
      for (int i = 0; i < 15; i++) {
        if (data_in[i] == 0x00) {
          check = data_in[i+1];
        }
      }
      printf("Signal Received\n");
    } else {
      printf("Nothing Received\n");
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  free(data_in);
}


void led(){
  while(1) {
    switch(check){
    case '1' :
      gpio_set_level(GREEN, 0);
      gpio_set_level(RED, 0);
      gpio_set_level(BLUE, 0);
    break;
    case '2' :
        gpio_set_level(GREEN, 0);
        gpio_set_level(RED, 1);
        gpio_set_level(BLUE, 0);
    break;
    case '3' :
        gpio_set_level(GREEN, 0);
        gpio_set_level(RED, 0);
        gpio_set_level(BLUE, 1);
    break;
    case '4' :
        gpio_set_level(GREEN, 1);
        gpio_set_level(RED, 0);
        gpio_set_level(BLUE, 0);
    break;

    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void hardware(){
  uint32_t io_num;
  while(1) {
    if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
      if(sstate == '1'){
        sstate = '3';
      } else if(sstate == '3'){
        sstate = '4';
      } else if(sstate == '4'){
        sstate = '2';
      } else if(sstate == '2'){
        sstate = '1';
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void app_main() {
uart_init();
hw_int_init();
rmt_tx_init();
led_init();


  xTaskCreate(recieve, "uart_rx_task", 1024*4, NULL, configMAX_PRIORITIES, NULL);
  xTaskCreate(transmit, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
  xTaskCreate(led, "set_gpio_task", 1024*2, NULL, configMAX_PRIORITIES-3, NULL);
  xTaskCreate(hardware, "button_task", 1024*2, NULL, configMAX_PRIORITIES-3, NULL);
}
