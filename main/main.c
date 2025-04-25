#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "semphr.h"
#include "mpu6050.h"
#include "hc06.h"
#include "Fusion.h"


#define UART_ID           uart0
#define BAUD_RATE         115200
#define UART_TX_PIN       0
#define UART_RX_PIN       1

#define PIN_POWER       16    // botão liga/desliga
#define PIN_LED         14    // LED indicador
#define DEBOUNCE_MS     50

#define SAMPLE_MS         10
#define SAMPLE_PERIOD     (SAMPLE_MS/1000.0f)
#define MPU_ADDRESS       0x68

#define CLICK_THRESHOLD   16384
// GPIOs digitais
#define PIN_ROTATE        6      // ↑
#define PIN_MOVE_RIGHT    7      // →
#define PIN_MOVE_LEFT     8      // ←
#define PIN_HARD_DROP     9      // SPACE
#define PIN_LED_BLUE           16   // blue status LED
#define PIN_LED_RED            19   // red status LED

// HC-06 “STATE” pin: HIGH when Bluetooth is connected
#define PIN_BT_STATE           10

// VBUS detect: HIGH when USB cable (PC) is plugged in
volatile bool usb_connected_flag = false;


typedef struct {
    uint8_t axis;
    int16_t val;
} event_t;

static QueueHandle_t  xQueuePos;

static SemaphoreHandle_t xPowerSem;
static TaskHandle_t     xMpuHandle, xDigitalHandle, xUartHandle;
bool usb_connected = false; // Flag global


void hc06_task(void *p) {
    uart_init(HC06_UART_ID, HC06_BAUD_RATE);
    gpio_set_function(HC06_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(HC06_RX_PIN, GPIO_FUNC_UART);
    hc06_init("TETRISTETRITETRIS ", "0000");

    while (true) {
        uart_puts(HC06_UART_ID, "OLAAA ");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


// ——————————————
// ISR do botão power
// ——————————————
void power_isr(uint gpio, uint32_t events) {
    BaseType_t woken = pdFALSE;
    // dá o semáforo para a power_task
    xSemaphoreGiveFromISR(xPowerSem, &woken);
    portYIELD_FROM_ISR(woken);
}

// ——————————————
// power_task: toggle on/off via semáforo
// ——————————————
static void power_task(void *p) {
    bool system_on = false;

    // configura LED
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 0);

    for (;;) {
        // aguarda ISR
        xSemaphoreTake(xPowerSem, portMAX_DELAY);

        // debounce
        vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));
        // confirma que continua pressionado (nível baixo)
        if (gpio_get(PIN_POWER) == 0) {
            system_on = !system_on;
            gpio_put(PIN_LED, system_on ? 1 : 0);

            if (system_on) {
                // liga as tasks de leitura
                vTaskResume(xMpuHandle);
                vTaskResume(xDigitalHandle);
                vTaskResume(xUartHandle);
            } else {
                // desliga as tasks de leitura
                vTaskSuspend(xMpuHandle);
                vTaskSuspend(xDigitalHandle);
                vTaskSuspend(xUartHandle);
            }
        }
    }
}

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

    // cria fila e semáforo
    xQueuePos  = xQueueCreate(16, sizeof(event_t));
    xPowerSem  = xSemaphoreCreateBinary();

    // configura pino power e ISR
    gpio_init(PIN_POWER);
    gpio_set_dir(PIN_POWER, GPIO_IN);
    gpio_pull_up(PIN_POWER);

    // gpio_init(PIN_LED_BLUE);
    // gpio_set_dir(PIN_LED_BLUE, GPIO_OUT);
    // gpio_put(PIN_LED_BLUE, 0);

    // gpio_init(PIN_LED_RED);
    // gpio_set_dir(PIN_LED_RED, GPIO_OUT);
    // gpio_put(PIN_LED_RED, 0);

    // // status inputs
    // gpio_init(PIN_BT_STATE);
    // gpio_set_dir(PIN_BT_STATE, GPIO_IN);
    // gpio_pull_down(PIN_BT_STATE);

    // gpio_init(HC06_ENABLE_PIN);
    // gpio_set_dir(HC06_ENABLE_PIN, GPIO_OUT);


    gpio_set_irq_enabled_with_callback(
        PIN_POWER,
        GPIO_IRQ_EDGE_FALL,   // detecta press
        true,
        power_isr
    );

    // cria tasks e armazena handles
    xTaskCreate(mpu6050_task,  "MPU",    2048, NULL, 1, &xMpuHandle);
    xTaskCreate(digital_task,  "DIG",    2048, NULL, 1, &xDigitalHandle);
    xTaskCreate(uart_task,     "UART",   2048, NULL, 1, &xUartHandle);

    // inicialmente, mantemos as tasks de leitura SUSPENSAS
    vTaskSuspend(xMpuHandle);
    vTaskSuspend(xDigitalHandle);
    vTaskSuspend(xUartHandle);

    // por fim, cria a power_task (prioridade ligeiramente maior)
    xTaskCreate(hc06_task, "UART_Task 1", 4096, NULL, 1, NULL);
    xTaskCreate(power_task,    "POWER",  512,  NULL, 2, NULL);

    vTaskStartScheduler();
    while (1) { tight_loop_contents(); }
    return 0;
}
