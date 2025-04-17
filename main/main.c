#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"
#include "Fusion.h"

#define UART_ID           uart0
#define BAUD_RATE         115200
#define UART_TX_PIN       0
#define UART_RX_PIN       1

#define SAMPLE_MS         10
#define SAMPLE_PERIOD     (SAMPLE_MS/1000.0f)

#define CLICK_THRESHOLD   24576
// GPIOs digitais
#define PIN_ROTATE        10      // ↑
#define PIN_MOVE_RIGHT    11      // →
#define PIN_MOVE_LEFT     12      // ←
#define PIN_HARD_DROP     13      // SPACE


typedef struct {
    uint8_t axis;
    int16_t val;
} event_t;

static QueueHandle_t  xQueuePos;

static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}

void mpu6050_task(void *p) {
    i2c_init(i2c_default, 400*1000);
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4); gpio_pull_up(5);

    mpu6050_reset();
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    int16_t accel[3], gyro[3], temp;
    bool clicked = false;

    while (1) {
        mpu6050_read_raw(accel, gyro, &temp);

        // detecta “cutucada” no X
        if (abs(accel[0]) > CLICK_THRESHOLD) {
            if (!clicked) {
                event_t ev = { .axis = 2, .val = 1 };
                xQueueSend(xQueuePos, &ev, 0);
                clicked = true;
            }
        } else {
            clicked = false;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void digital_task(void *p) {
    const int pins[4] = { PIN_ROTATE, PIN_MOVE_RIGHT, PIN_MOVE_LEFT, PIN_HARD_DROP };
    for (int i = 0; i < 4; i++) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_IN);
        gpio_pull_up(pins[i]);
    }
    bool last[4] = {false};

    while (1) {
        for (int i = 0; i < 4; i++) {
            bool pressed = !gpio_get(pins[i]);
            if (pressed && !last[i]) {
                event_t ev = { .axis = (uint8_t)(3 + i), .val = 1 };
                xQueueSend(xQueuePos, &ev, 0);
            }
            last[i] = pressed;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void uart_task(void *p) {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    event_t ev;
    uint8_t pkt[4];
    while (1) {
        if (xQueueReceive(xQueuePos, &ev, portMAX_DELAY)) {
            pkt[0] = 0xFF;
            pkt[1] = ev.axis;
            pkt[2] = ev.val & 0xFF;
            pkt[3] = (ev.val >> 8) & 0xFF;
            uart_write_blocking(UART_ID, pkt, 4);
        }
    }
}

int main() {
    stdio_init_all();
    xQueuePos = xQueueCreate(16, sizeof(event_t));

    xTaskCreate(mpu6050_task,  "MPU", 2048, NULL, 1, NULL);
    xTaskCreate(digital_task,  "DIG", 2048, NULL, 1, NULL);
    xTaskCreate(uart_task,     "UART",2048, NULL, 1, NULL);

    vTaskStartScheduler();
    while (1) {}
    return 0;
}
