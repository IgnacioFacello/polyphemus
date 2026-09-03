
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "hal/adc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to a potentiometer device instance.
 */
typedef struct pot_dev_t *pot_handle_t;

/**
 * @brief Configuration used to initialize a potentiometer device.
 */
typedef struct {
    adc_unit_t unit;            ///< ADC unit, e.g. ADC_UNIT_1
    adc_channel_t channel;      ///< ADC channel, e.g. ADC_CHANNEL_7 (check pinout for GPIO)
    adc_bitwidth_t bitwidth;    ///< ADC resolution, e.g. ADC_BITWIDTH_12 (0-4095)
    adc_atten_t atten;          ///< ADC attenuation, e.g. ADC_ATTEN_DB_6
    float max_voltage;          ///< Full-scale voltage for the chosen attenuation (e.g. 2.2f for DB_6)
    uint32_t avg_samples;       ///< Number of raw samples averaged per pot_update() call
} pot_config_t;

/**
 * @brief Initialize a potentiometer device and its underlying ADC unit/channel.
 *
 * @param config      Configuration describing the ADC unit/channel to use.
 * @param out_handle  Output handle, valid on ESP_OK.
 * @return esp_err_t  ESP_OK on success, ESP_ERR_INVALID_ARG on bad input,
 *                     or an error from the underlying ADC driver.
 */
esp_err_t pot_init(const pot_config_t *config, pot_handle_t *out_handle);

/**
 * @brief Release resources associated with a potentiometer device.
 */
esp_err_t pot_deinit(pot_handle_t handle);

/**
 * @brief Take avg_samples ADC readings, average them, and update the
 *        device's internal accumulator plus its running min/max range.
 *
 * Call this periodically (e.g. from a timer) before reading position/voltage.
 */
esp_err_t pot_update(pot_handle_t handle);

/**
 * @brief Get the last averaged raw ADC reading (0..2^bitwidth-1 range).
 */
esp_err_t pot_get_raw(pot_handle_t handle, float *out_raw);

/**
 * @brief Get the potentiometer position as a percentage (0-100),
 *        scaled against the observed min/max raw range so far.
 */
esp_err_t pot_get_percentage(pot_handle_t handle, float *out_percentage);

/**
 * @brief Get the potentiometer position converted to volts.
 */
esp_err_t pot_get_voltage(pot_handle_t handle, float *out_voltage);

#ifdef __cplusplus
}
#endif
