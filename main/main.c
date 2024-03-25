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
#define DEAD_ZONE 170

QueueHandle_t xQueueAdc;

typedef struct adc {
    char axis;
    int val;
} adc_t;

void write_package(adc_t data) {
    int val = data.val;
    int msb = val >> 8;
    int lsb = val & 0xFF;

    uart_putc_raw(uart0, data.axis);
    uart_putc_raw(uart0, msb);
    uart_putc_raw(uart0, lsb);
    uart_putc_raw(uart0, -1);
}

void uart_task(void *p) {
    adc_t data;
    uint8_t sync_byte = 0xFF; // Byte de sincronização

    while (1) {
        xQueueReceive(xQueueAdc, &data, portMAX_DELAY);

        write_package(data); // Envio dos dados
    }
}

int transform_value(int val) {
    val -= 2048;                // Mudança de escala
    val /= 8;                   // Divisão por 8
    if (abs(val) < DEAD_ZONE) { // Zona morta
        return 0;
    } else {
        return val / 100;
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
        int raw_val = adc_read();            // Leitura do valor do joystick no eixo X
        data.val = transform_value(raw_val); // Transformação do valor
        if (data.val != 0)
            xQueueSend(xQueueAdc, &data, portMAX_DELAY); // Envio dos dados para a fila
        vTaskDelay(pdMS_TO_TICKS(100));                  // Delay para evitar sobrecarga
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
        int raw_val = adc_read();            // Leitura do valor do joystick no eixo Y
        data.val = transform_value(raw_val); // Transformação do valor
        if (data.val != 0)
            xQueueSend(xQueueAdc, &data, portMAX_DELAY); // Envio dos dados para a fila
        vTaskDelay(pdMS_TO_TICKS(100));                  // Delay para evitar sobrecarga
    }
}

void main() {
    stdio_init_all();
    uart_init(uart0, 9600);

    // Inicialização do ADC e dos pinos do joystick
    adc_init();
    adc_gpio_init(JOY_X_PIN);
    adc_gpio_init(JOY_Y_PIN);

    xQueueAdc = xQueueCreate(32, sizeof(adc_t));

    xTaskCreate(uart_task, "uart_task", 4096, NULL, 1, NULL);
    xTaskCreate(x_task, "x_task", 4096, NULL, 1, NULL);
    xTaskCreate(y_task, "y_task", 4096, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true)
        ;
}