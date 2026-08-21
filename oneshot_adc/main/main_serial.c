#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>
#include "driver/gpio.h"

/*
 * 74HC595 pin mapping
 *
 * IMPORTANT: GPIO34, 35, 36 (SVP), and 39 (SVN) are INPUT-ONLY on the
 * ESP32 — they cannot drive DS, OE, RCLK, or SRCLK. Only G32 works as
 * wired in the original request. The pins below assume you've moved
 * those four lines to regular GPIOs. Change the defines to match your
 * actual wiring.
 */
#define PIN_DS    GPIO_NUM_25   // Serial In (DS)          - was SP/GPIO36
#define PIN_OE    GPIO_NUM_26   // Output Enable, active LOW - was SM/GPIO39
#define PIN_RCLK  GPIO_NUM_27   // Register/Latch Clock (STCP) - was G34
#define PIN_SRCLK GPIO_NUM_33   // Serial/Shift Clock (SHCP)   - was G35
#define PIN_MR    GPIO_NUM_32   // Master Reset, active LOW (as wired)

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  ((byte) & 0x80 ? '1' : '0'), \
  ((byte) & 0x40 ? '1' : '0'), \
  ((byte) & 0x20 ? '1' : '0'), \
  ((byte) & 0x10 ? '1' : '0'), \
  ((byte) & 0x08 ? '1' : '0'), \
  ((byte) & 0x04 ? '1' : '0'), \
  ((byte) & 0x02 ? '1' : '0'), \
  ((byte) & 0x01 ? '1' : '0')

static void hc595_gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_DS) | (1ULL << PIN_OE) |
                         (1ULL << PIN_RCLK) | (1ULL << PIN_SRCLK) |
                         (1ULL << PIN_MR),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    gpio_set_level(PIN_MR, 1);    // not in reset
    gpio_set_level(PIN_OE, 0);    // outputs enabled (active low)
    gpio_set_level(PIN_RCLK, 0);
    gpio_set_level(PIN_SRCLK, 0);
    gpio_set_level(PIN_DS, 0);
}

// Shifts one byte out MSB-first and latches it to the output register.
static void hc595_write(uint8_t value)
{
    for (int i = 7; i >= 0; i--) {
        gpio_set_level(PIN_DS, (value >> i) & 0x01);
        gpio_set_level(PIN_SRCLK, 1);
        gpio_set_level(PIN_SRCLK, 0);
    }
    // Latch shift register contents into the storage register / outputs
    gpio_set_level(PIN_RCLK, 1);
    gpio_set_level(PIN_RCLK, 0);
}

// Pulses the active-low master reset, clearing the shift register.
static void hc595_clear(void)
{
    gpio_set_level(PIN_MR, 0);
    gpio_set_level(PIN_MR, 1);
    hc595_write(0x00);
}

void app_main(void)
{
    hc595_gpio_init();
    hc595_clear();

    uint8_t value = 1;
    while (1) {
        hc595_write(value);
        ESP_LOGI("NOTICE", BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(value));
        value *= 2;
        if (value == 0){
            value = 1;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
