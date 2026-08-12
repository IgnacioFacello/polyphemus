/*
 * encoder.h — Quadrature encoder driver built on the ESP-IDF PCNT peripheral.
 *
 * Supports multiple independent encoder instances via an opaque handle.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Opaque handle to an encoder instance. */
typedef struct encoder_dev_t *encoder_handle_t;

/* Optional: called from a library-owned task (NOT from ISR context) whenever
 * the pulse count crosses one of the internal watch points. Safe to do
 * logging, mutex-protected work, etc. inside this callback. */
typedef void (*encoder_event_cb_t)(int watch_point_value, void *user_ctx);

typedef struct {
    int      gpio_a;              /* Encoder channel A GPIO */
    int      gpio_b;              /* Encoder channel B GPIO */
    int      pcnt_high_limit;     /* Counter wraps/saturates above this */
    int      pcnt_low_limit;      /* Counter wraps/saturates below this */
    uint32_t glitch_filter_ns;    /* Software glitch filter, 0 to disable */
} encoder_config_t;

/* Sensible defaults: +/-32767 range, 1us glitch filter. */
#define ENCODER_DEFAULT_CONFIG(a_gpio, b_gpio, ppr)   \
    {                                             \
        .gpio_a = (a_gpio),                       \
        .gpio_b = (b_gpio),                       \
        .pcnt_high_limit = ppr*4,                  \
        .pcnt_low_limit = -ppr*4,                  \
        .glitch_filter_ns = 1000,                  \
    }

/* Create and start an encoder instance. */
esp_err_t encoder_init(const encoder_config_t *config, encoder_handle_t *out_handle);

/* Stop and free an encoder instance. */
esp_err_t encoder_deinit(encoder_handle_t handle);

/* Read the current raw pulse count. */
esp_err_t encoder_get_count(encoder_handle_t handle, int *count);

/* Reset the counter to zero. */
esp_err_t encoder_clear_count(encoder_handle_t handle);

#ifdef __cplusplus
}
#endif
