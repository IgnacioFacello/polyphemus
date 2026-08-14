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

typedef struct {
    int      gpio_a;              /* Encoder channel A GPIO */
    int      gpio_b;              /* Encoder channel B GPIO */
    int      pcnt_high_limit;     /* Counter wraps back near 0 above this */
    int      pcnt_low_limit;      /* Counter wraps back near 0 below this */
    uint32_t glitch_filter_ns;    /* Software glitch filter, 0 to disable */
} encoder_config_t;

/* Sensible defaults: +/-ppr range, 1us glitch filter. */
#define ENCODER_DEFAULT_CONFIG(a_gpio, b_gpio, ppr)   \
    {                                             \
        .gpio_a = (a_gpio),                       \
        .gpio_b = (b_gpio),                       \
        .pcnt_high_limit = ppr,                  \
        .pcnt_low_limit = -ppr,                  \
        .glitch_filter_ns = 1000,                  \
    }

/* Create and start an encoder instance. Internally registers watch points at
 * pcnt_high_limit/pcnt_low_limit so wraparound can be tracked — see
 * encoder_get_total_count(). */
esp_err_t encoder_init(const encoder_config_t *config, encoder_handle_t *out_handle);

/* Stop and free an encoder instance. */
esp_err_t encoder_deinit(encoder_handle_t handle);

/* Read the current raw pulse count (wraps at pcnt_high_limit/pcnt_low_limit).
 * Do NOT use this directly for RPM/velocity calculations — a wrap produces a
 * large discontinuous jump. Use encoder_get_total_count() instead. */
esp_err_t encoder_get_count(encoder_handle_t handle, int *count);

/* Read an unwrapped, continuous total count: the raw hardware count plus an
 * internally tracked offset that compensates for every wrap at
 * pcnt_high_limit/pcnt_low_limit. Safe to use for RPM/velocity calculations —
 * delta_total_count / delta_time stays correct across wraps. */
esp_err_t encoder_get_total_count(encoder_handle_t handle, int64_t *total_count);

/* Reset both the raw hardware counter and the internal wrap offset to zero. */
esp_err_t encoder_clear_count(encoder_handle_t handle);

#ifdef __cplusplus
}
#endif
