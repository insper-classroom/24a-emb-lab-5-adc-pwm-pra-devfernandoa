/*
 * LED blink with FreeRTOS
 */
#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>
#include <task.h>

#include "hardware/adc.h"
#include "pico/stdlib.h"
#include <stdio.h>

#include <math.h>
#include <stdlib.h>

#define UART_ID uart0

#define JOY_X_PIN 26
#define JOY_Y_PIN 27
#define DEAD_ZONE 100

QueueHandle_t xQueueAdc;

typedef struct adc {
    int axis;
    int val;
} adc_t;

void uart_task(void *p) {
    adc_t data;
    uint8_t sync_byte = 0xFF; // Byte de sincronização

    while (1) {
        xQueueReceive(xQueueAdc, &data, portMAX_DELAY);

        // Envio do byte de sincronização
        uart_puts(UART_ID, &sync_byte);

        // Formatação dos dados
        uint8_t buffer[4];
        buffer[0] = data.axis;
        buffer[1] = data.val >> 8;   // MSB
        buffer[2] = data.val & 0xFF; // LSB
        buffer[3] = -1;              // EOP

        // Envio dos dados pela UART
        uart_puts(UART_ID, buffer);
    }
}

void x_task(void *p) {
    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_t data;
    data.axis = 0; // Eixo X

    while (1) {
        adc_select_input(0);
        data.val = adc_read();                       // Leitura do valor do joystick no eixo X
        xQueueSend(xQueueAdc, &data, portMAX_DELAY); // Envio dos dados para a fila
        vTaskDelay(pdMS_TO_TICKS(10));               // Delay para evitar sobrecarga
    }
}

void y_task(void *p) {
    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_t data;
    data.axis = 1; // Eixo Y

    while (1) {
        adc_select_input(1);
        data.val = adc_read();                       // Leitura do valor do joystick no eixo Y
        xQueueSend(xQueueAdc, &data, portMAX_DELAY); // Envio dos dados para a fila
        vTaskDelay(pdMS_TO_TICKS(10));               // Delay para evitar sobrecarga
    }
}

int main() {
    stdio_init_all();

    xQueueAdc = xQueueCreate(32, sizeof(adc_t));

    xTaskCreate(uart_task, "uart_task", 4096, NULL, 1, NULL);
    xTaskCreate(x_task, "x_task", 4096, NULL, 1, NULL);
    xTaskCreate(y_task, "y_task", 4096, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true)
        ;
}