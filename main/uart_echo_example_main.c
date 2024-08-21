/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "string.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "C:/Users/ADMIN/VS_Projects/uart_DWIN_esp32c3/build/config/sdkconfig.h"

#define RS485

#define ECHO_TEST_TXD 4
#define ECHO_TEST_RXD 5
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM (CONFIG_EXAMPLE_UART_PORT_NUM)
#define ECHO_UART_BAUD_RATE (CONFIG_EXAMPLE_UART_BAUD_RATE)
#define ECHO_TASK_STACK_SIZE (CONFIG_EXAMPLE_TASK_STACK_SIZE)

#define RST_DWIN_CMD "\x5a\xa5\x07\x82\x00\x04\x55\xaa\x5a\xa5"
#define SWITCH_PAGE "\x5a\xa5\x07\x82\x00\x84\x5a\x01\x00\x01"
#define FLOW_ACUM_COM "\x0F\x03\x02\x04\x00\x04\x05\x5E"
#define FLOW_INST_COM "\x0F\x03\x02\x09\x00\x02\x14\x9F"
#define DATE_COM "\x0F\x03\x02\x0C\x00\x06\x05\x5D"

#define LOG_EN 0

#define RX_BUF_SIZE 1024
#define TX_BUF_SIZE 1024

#define PIN_SWITCH 9

#ifdef RS485
#define PIN_DE GPIO_NUM_3
#endif

#define PATTERN_LEN 3

#define TAG "UART"

QueueHandle_t uart_queue;

bool once = 0;
int level;
char addr_low, addr_hi, cmd;
uint16_t addr = 0;
uint8_t n_bytes;
uint64_t combined_value = 0;
int len;
float acum;

char incoming_message[RX_BUF_SIZE];

void print_data(char *, uint8_t);

void uart_event_task(void *params)
{
    uart_event_t uart_event;
    uint8_t *received_buffer = malloc(RX_BUF_SIZE);
    size_t datalen;
    while (true)
    {
        if (xQueueReceive(uart_queue, &uart_event, portMAX_DELAY))
        {
            switch (uart_event.type)
            {
            case UART_DATA:
                uart_read_bytes(ECHO_UART_PORT_NUM, (uint8_t *)incoming_message, RX_BUF_SIZE, pdMS_TO_TICKS(500));
                n_bytes = incoming_message[3];
                addr_hi = incoming_message[4];
                addr_low = incoming_message[5];
                addr = addr_low | addr_hi << 8;
                // printf("0x%02X ", n_bytes);
                // printf("0x%02X ", incoming_message[4]);
                // printf("0x%02X ", incoming_message[5]);
                // printf("0x%04X ", addr);
                // printf("0x%02X ", incoming_message[n_bytes + 2]);
                // printf("\n");
                // ESP_LOGI(TAG, "UART_DATA");
                // printf("\n");
                // print_data(incoming_message, n_bytes);
                // printf("0x%02X ", n_bytes);
                // printf("\n");

                for (int i = 4; i < 12; i++)
                {
                    // printf("0x%02X ", incoming_message[i]);
                    combined_value |= ((uint64_t)incoming_message[i] << ((7 - i) * 8));
                }
                // printf("\n");
                // printf("acum: 0x%016llx\n", combined_value);
                // printf("acum: %lld\n", combined_value);
                printf("acum: %f\n", (float)combined_value / 1000000);
                // switch (addr_hi)
                // {
                // case /* constant-expression */:
                //     /* code */
                //     break;

                // default:
                //     break;
                // }
                break;
            case UART_BREAK:
                // ESP_LOGI(TAG, "UART_BREAK");
                break;
            case UART_BUFFER_FULL:
                ESP_LOGI(TAG, "UART_BUFFER_FULL");
                break;
            case UART_FIFO_OVF:
                ESP_LOGI(TAG, "UART_FIFO_OVF");
                uart_flush_input(UART_NUM_1);
                xQueueReset(uart_queue);
                break;
            case UART_FRAME_ERR:
                ESP_LOGI(TAG, "UART_FRAME_ERR");
                break;
            case UART_PARITY_ERR:
                ESP_LOGI(TAG, "UART_PARITY_ERR");
                break;
            case UART_DATA_BREAK:
                ESP_LOGI(TAG, "UART_DATA_BREAK");
                break;
            case UART_PATTERN_DET:
                ESP_LOGI(TAG, "UART_PATTERN_DET");
                uart_get_buffered_data_len(UART_NUM_1, &datalen);
                int pos = uart_pattern_pop_pos(UART_NUM_1);
                ESP_LOGI(TAG, "Detected %d pos %d", datalen, pos);
                uart_read_bytes(UART_NUM_1, received_buffer, datalen - PATTERN_LEN, pdMS_TO_TICKS(100));
                uint8_t pat[PATTERN_LEN + 1];
                memset(pat, 0, sizeof(pat));
                uart_read_bytes(UART_NUM_1, pat, PATTERN_LEN, pdMS_TO_TICKS(100));
                printf("data: %.*s === pattern: %s\n", datalen - PATTERN_LEN, received_buffer, pat);
                break;
            default:
                break;
            }
        }
    }
}

void init_gpio(void)
{
#ifdef RS485
    gpio_set_direction(PIN_DE, GPIO_MODE_OUTPUT);
#endif
    gpio_set_direction(PIN_SWITCH, GPIO_MODE_INPUT);
    gpio_pullup_en(PIN_SWITCH);
}

void print_data(char *message, uint8_t n_frame)
{
    for (int i = 0; i < (n_frame + 6); i++)
    {
        printf("0x%X ", message[i]);
    }
    printf("\n");
}

void app_main(void)
{
    init_gpio();

    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_param_config(ECHO_UART_PORT_NUM, &uart_config);
    uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE, TX_BUF_SIZE, 20, &uart_queue, 0);

#ifndef RS485

    char message[] = RST_DWIN_CMD;

    print_data(message);
    // printf("sending: 0x%X\n", message);
    uart_write_bytes(ECHO_UART_PORT_NUM, message, sizeof(message));

    memset(incoming_message, 0, sizeof(incoming_message));
    uart_read_bytes(ECHO_UART_PORT_NUM, (uint8_t *)incoming_message, RX_BUF_SIZE, pdMS_TO_TICKS(500));
    // printf("received: 0x%X\n", incoming_message);
    print_data(incoming_message);
#endif
    // uart_enable_pattern_det_intr(UART_NUM_1, '+', PATTERN_LEN, 10000, 10, 10);

    uart_pattern_queue_reset(UART_NUM_1, 20);
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 10, NULL);

    while (true)
    {
        level = gpio_get_level(PIN_SWITCH);
        // if (level && !once)
        // {
        //     printf("IN: 1\n");
        //     once = 1;
        // }
//         if (!level && once)
//         {

//             printf("IN: 0\n");
//             once = 0;
#ifndef RS485
        char message[] = SWITCH_PAGE;
#else
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        gpio_set_level(PIN_DE, 1);
        char message[] = "\x0F\x03\x02\x04\x00\x04\x05\x5E"; // 0F 03 02 04 00 04 05 5E
#endif
        uart_write_bytes(ECHO_UART_PORT_NUM, message, (sizeof(message) - 1));
        // print_data(message, 2);
        // }
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(PIN_DE, 0);
        memset(incoming_message, 0, sizeof(incoming_message));
        len = uart_read_bytes(ECHO_UART_PORT_NUM, (uint8_t *)incoming_message, RX_BUF_SIZE, pdMS_TO_TICKS(500));

        // else
        // {
        //     print_data(incoming_message);
        // }
    }
}
