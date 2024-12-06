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
#define ALL_DATA "\x0F\x03\x02\x00\x00\x19\x84\x96"

#define LOG_EN

#define RX_BUF_SIZE 1024
#define TX_BUF_SIZE 1024

#define PIN_SWITCH 9

#ifdef RS485
#define PIN_DE GPIO_NUM_3
#endif

#define PATTERN_LEN 3

#define TAG "UART"

int j;

typedef enum
{
    DATE_COM_STATE,
    FLOW_INST_COM_STATE,
    FLOW_ACUM_COM_STATE,
} comm_state_t;

QueueHandle_t uart_queue;

bool once = 0;
int level;
char addr_low, addr_hi, cmd;
uint16_t addr = 0;
uint8_t n_bytes;
uint64_t combined_value = 0;
uint32_t flow_result = 0;
int len;
float acum;

static comm_state_t comm_state = FLOW_ACUM_COM_STATE;

char incoming_message[RX_BUF_SIZE];
char message[9] = FLOW_ACUM_COM;

void print_data(char *, uint8_t);

esp_err_t send_command(uint8_t);

esp_err_t rcv_command(uint8_t);

esp_err_t calc_flow(int, int);

esp_err_t proccess_command();

/**
 * @brief Recibe mensajes devueltos desde el caudalimetro
 * @param [in] n_bytes Numero de bytes para mandar a funcion print_data
 * @return esp_err_t Estado de la operacion (ESP_OK si todo fue bien, ESP_FAIL si hubo algun problema)
 */

esp_err_t rcv_command(uint8_t n_bytes)
{
#ifdef LOG_EN
    ESP_LOGI("FUNCION DE RECEPCION", "FUNCION DE RECEPCION\n");
#endif

    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(PIN_DE, 0);
    memset(incoming_message, 0, sizeof(incoming_message));
    len = uart_read_bytes(ECHO_UART_PORT_NUM, (uint8_t *)incoming_message, RX_BUF_SIZE, pdMS_TO_TICKS(500));
#ifdef LOG_EN
    print_data(incoming_message, len);
#endif
    return ESP_OK;
}

/**
 * @brief Manda los comandos al caudalimetro
 * @param [in] cmd Comando a enviar al caudalimetro por puerto serie
 * @param [in] n_bytes Numero de bytes para mandar a funcion print_data
 * @return esp_err_t Estado de la operacion (ESP_OK si todo fue bien, ESP_FAIL si hubo algun problema)
 */

esp_err_t send_command(uint8_t n_bytes)
{
    gpio_set_level(PIN_DE, 1);
    uart_write_bytes(ECHO_UART_PORT_NUM, message, (sizeof(message) - 1));
#ifdef LOG_EN
    print_data(message, n_bytes);
#endif
    ESP_ERROR_CHECK(rcv_command(n_bytes));
    return ESP_OK;
}

/**
 * @brief
 *
 */

esp_err_t calc_flow(int first, int last)
{
    combined_value = 0;
    for (int i = first; i < last; i++)
    {
        if (comm_state == FLOW_ACUM_COM_STATE)
            combined_value = (combined_value << 8) | incoming_message[i];
        if (comm_state == FLOW_INST_COM_STATE)
            flow_result |= ((uint32_t)incoming_message[i] << ((3 - i + first) * 8));
    }
    return ESP_OK;
}

/**
 * @brief Tarea para manejo del puerto serie
 * @param [in] params los parametros de la Task creada
 * @return void
 */

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
#ifndef RS485
                addr_hi = incoming_message[4];
                addr_low = incoming_message[5];
                addr = addr_low | addr_hi << 8;
                printf("0x%02X ", n_bytes);
                printf("0x%02X ", incoming_message[4]);
                printf("0x%02X ", incoming_message[5]);
                printf("0x%04X ", addr);
                printf("0x%02X ", incoming_message[n_bytes + 2]);
                printf("\n");
                ESP_LOGI(TAG, "UART_DATA");
                printf("\n");
                print_data(incoming_message, n_bytes);
                printf("0x%02X ", n_bytes);
                printf("\n");
#endif

                // pasar a funcion:
                switch (comm_state)
                {
                case FLOW_ACUM_COM_STATE:
                    break;
                case FLOW_INST_COM_STATE:
                    break;
                case DATE_COM_STATE:
                    break;
                }

                break;
            case UART_BREAK:
#ifdef LOG_EN
                ESP_LOGI(TAG, "UART_BREAK");
#endif
                break;
            case UART_BUFFER_FULL:
#ifdef LOG_EN
                ESP_LOGI(TAG, "UART_BUFFER_FULL");
#endif
                break;
            case UART_FIFO_OVF:
#ifdef LOG_EN
                ESP_LOGI(TAG, "UART_FIFO_OVF");
#endif
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

/**
 * @brief Inicializa entradas y salidas
 * @param [in] void
 * @return void
 */

void init_gpio(void)
{
#ifdef RS485
    gpio_set_direction(PIN_DE, GPIO_MODE_OUTPUT);
#endif
    gpio_set_direction(PIN_SWITCH, GPIO_MODE_INPUT);
    gpio_pullup_en(PIN_SWITCH);
}

/**
 * @brief imprime al terminal lo que envia y recibe del caudalimetro (en formato hexadecimal)
 * @param [in] message Mensaje a imprimir en terminal
 * @param [in] n_frame Cantidad de bytes a mandar
 * @return void
 */

void print_data(char *message, uint8_t n_frame)
{
    for (int i = 0; i < (n_frame); i++)
    {
        printf("0x%X ", message[i]);
    }
    printf("\n");
}

void app_main(void)
{
    init_gpio();

    uart_config_t uart_config = {
        //.baud_rate = ECHO_UART_BAUD_RATE,
        .baud_rate = 2400,
        .data_bits = UART_DATA_8_BITS,
        //.parity = UART_PARITY_DISABLE,
        .parity = UART_PARITY_EVEN,
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

    uart_pattern_queue_reset(UART_NUM_1, 20);
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 10, NULL);

    while (true)
    {
        level = gpio_get_level(PIN_SWITCH);
#ifdef IN_ACT
        if (level && !once)
        {
            printf("IN: 1\n");
            once = 1;
        }
        if (!level && once)
        {

            printf("IN: 0\n");
            once = 0;
        }
#endif
#ifndef RS485
        char message[] = SWITCH_PAGE;
#else
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        switch (comm_state)
        {
        case FLOW_ACUM_COM_STATE:
            memcpy(message, FLOW_ACUM_COM, sizeof(FLOW_ACUM_COM));
            ESP_ERROR_CHECK(send_command(8));
            calc_flow(3, 11);
            printf("acum: %f\n", (float)combined_value / 1000);
            comm_state = FLOW_INST_COM_STATE;
            break;
        case FLOW_INST_COM_STATE:
            memcpy(message, FLOW_INST_COM, sizeof(FLOW_INST_COM));
            ESP_ERROR_CHECK(send_command(8));
            calc_flow(3, 7);
            printf("flow: %f\n", (float)flow_result * 16.6 / 1000000);
            comm_state = DATE_COM_STATE;
            break;
        case DATE_COM_STATE:
            memcpy(message, DATE_COM, sizeof(DATE_COM));
            ESP_ERROR_CHECK(send_command(8));
            comm_state = FLOW_ACUM_COM_STATE;
            break;
        }

#endif
    }
}
