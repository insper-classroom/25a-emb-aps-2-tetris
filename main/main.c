#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

typedef struct {
    int axis;   // 0 = X, 1 = Y, 2 = Botão
    int val;    // valor filtrado ou estado do botão
} adc_data_t;

#define BTN_PIN        18

#define ADC_X          26
#define ADC_Y          27

#define UART_ID        uart0
#define UART_TX_PIN    1
#define UART_RX_PIN    2

#define ZERO_OFFSET    2048
#define FULLSCALE      2048
#define SCALE_MAX      255
#define ZONA_MORTA     30

#define WINDOW_SIZE    5
#define SAMPLE_DELAY_MS 300
#define DEBOUNCE_MS    50

QueueHandle_t xQueueADC;

static int filtrar_valor(int raw_value) {
    int offset = raw_value - ZERO_OFFSET;
    int scaled = (offset * SCALE_MAX) / FULLSCALE;
    if (abs(scaled) < ZONA_MORTA) scaled = 0;
    if (scaled > SCALE_MAX)  scaled = SCALE_MAX;
    if (scaled < -SCALE_MAX) scaled = -SCALE_MAX;
    return scaled;
}

static void x_task(void *pvParameters) {
    int buffer[WINDOW_SIZE] = {0};
    int soma = 0, index = 0;
    adc_data_t pacote;

    while (1) {
        adc_select_input(0);
        int leitura = adc_read();

        soma -= buffer[index];
        buffer[index] = leitura;
        soma += leitura;

        index = (index + 1) % WINDOW_SIZE;

        pacote.axis = 0;
        int media = soma / WINDOW_SIZE;
        pacote.val = filtrar_valor(media);

        xQueueSend(xQueueADC, &pacote, pdMS_TO_TICKS(200));
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_DELAY_MS));
    }
}

static void y_task(void *pvParameters) {
    int buffer[WINDOW_SIZE] = {0};
    int soma = 0, index = 0;
    adc_data_t pacote;

    while (1) {
        adc_select_input(1);
        int leitura = adc_read();

        soma -= buffer[index];
        buffer[index] = leitura;
        soma += leitura;

        index = (index + 1) % WINDOW_SIZE;

        pacote.axis = 1;
        int media = soma / WINDOW_SIZE;
        pacote.val = filtrar_valor(media);

        xQueueSend(xQueueADC, &pacote, pdMS_TO_TICKS(200));
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_DELAY_MS));
    }
}

static void button_task(void *pvParameters) {
    adc_data_t pacote;
    int last_state = gpio_get(BTN_PIN);
    int stable_state = last_state;
    TickType_t last_change = xTaskGetTickCount();

    while (1) {
        int cur = gpio_get(BTN_PIN);
        if (cur != stable_state) {
            last_change = xTaskGetTickCount();
            stable_state = cur;
        }
        // aguarda debounce
        if ((xTaskGetTickCount() - last_change) > pdMS_TO_TICKS(DEBOUNCE_MS)) {
            if (stable_state != last_state) {
                // houve mudança de estado
                pacote.axis = 2;
                // supondo pull-up: 0 = pressionado, 1 = solto
                pacote.val = (stable_state == 0) ? 1 : 0;
                xQueueSend(xQueueADC, &pacote, pdMS_TO_TICKS(200));
                last_state = stable_state;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void uart_task(void *pvParameters) {
    adc_data_t recebido;
    uint8_t pacote[4];

    while (1) {
        if (xQueueReceive(xQueueADC, &recebido, portMAX_DELAY) == pdTRUE) {
            int v = recebido.val;
            pacote[0] = 0xFF;                     // header
            pacote[1] = (uint8_t)recebido.axis;  // eixo ou botão
            pacote[2] = v & 0xFF;                // LSB
            pacote[3] = (v >> 8) & 0xFF;         // MSB

            uart_write_blocking(UART_ID, pacote, sizeof(pacote));
        }
    }
}

int main() {
    stdio_init_all();

    // inicialização ADC
    adc_init();
    adc_gpio_init(ADC_X);
    adc_gpio_init(ADC_Y);

    // inicialização UART
    uart_init(UART_ID, 115200);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // inicialização botão
    gpio_init(BTN_PIN);
    gpio_set_dir(BTN_PIN, GPIO_IN);
    gpio_pull_up(BTN_PIN);

    // fila para todos os pacotes
    xQueueADC = xQueueCreate(32, sizeof(adc_data_t));

    // criação de tarefas
    xTaskCreate(x_task,      "x_task",      4096, NULL, 1, NULL);
    xTaskCreate(y_task,      "y_task",      4096, NULL, 1, NULL);
    xTaskCreate(button_task, "button_task", 2048, NULL, 1, NULL);
    xTaskCreate(uart_task,   "uart_task",   4096, NULL, 1, NULL);

    vTaskStartScheduler();

    while (1) { }
    return 0;
}
