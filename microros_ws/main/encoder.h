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
#define ENCODER_DEFAULT_CONFIG(a_gpio, b_gpio, limit)   \
    {                                             \
        .gpio_a = (a_gpio),                       \
        .gpio_b = (b_gpio),                       \
        .pcnt_high_limit = limit,                  \
        .pcnt_low_limit = -limit,                  \
        .glitch_filter_ns = 1000,                  \
    }

/* Funcion de inicialización de encoders. Configura una unidad pcnt,
 * setea watchpoints y activa la misma
 * */
esp_err_t encoder_init(const encoder_config_t *config, encoder_handle_t *out_handle);

/* Detiene y libera la instancia del encoder */
esp_err_t encoder_deinit(encoder_handle_t handle);

/* Lee el valor actual del pcnt, este valor se reinicia a 0 al superar alguno de los limites */
esp_err_t encoder_get_count(encoder_handle_t handle, int *count);

/* Lee el valor acumulado de la posicion, no es suceptible a los limites del pcnt.
 * Usa un entero de 64 bits para el acumulador,
 * si no se reinicia periodicamente con clear_count puede generar overflow
 */
esp_err_t encoder_get_total_count(encoder_handle_t handle, int64_t *total_count);

/* Resetea tanto el valor del pcnt como el del acumulador */
esp_err_t encoder_clear_count(encoder_handle_t handle);

#ifdef __cplusplus
}
#endif
