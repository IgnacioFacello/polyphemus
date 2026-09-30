
#include "potentiometer.h"

#include <stdlib.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

static const char *TAG = "potentiometer";

struct pot_dev_t {
    adc_oneshot_unit_handle_t adc_handle;
    adc_channel_t channel;
    uint32_t avg_samples;
    float max_voltage;
    float bit_max;          // e.g. 4095.0f for 12-bit
    float accumulator;      // last averaged raw reading
    float raw_min;          // running min of averaged readings seen so far
    float raw_max;          // running max of averaged readings seen so far
    bool has_reading;       // whether pot_update() has produced a valid sample yet
};

static float bitwidth_to_max(adc_bitwidth_t bitwidth)
{
    switch (bitwidth) {
        case ADC_BITWIDTH_9:  return 511.0f;
        case ADC_BITWIDTH_10: return 1023.0f;
        case ADC_BITWIDTH_11: return 2047.0f;
        case ADC_BITWIDTH_12: return 4095.0f;
        case ADC_BITWIDTH_13: return 8191.0f;
        default:              return 4095.0f;
    }
}

esp_err_t pot_init(const pot_config_t *config, pot_handle_t *out_handle)
{
    if (config == NULL || out_handle == NULL || config->avg_samples == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    struct pot_dev_t *pot = calloc(1, sizeof(struct pot_dev_t));
    if (pot == NULL) {
        return ESP_ERR_NO_MEM;
    }
    adc_oneshot_unit_handle_t adc_handle;

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = config->unit,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
    };
    esp_err_t err = adc_oneshot_new_unit(&init_config, &adc_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ADC unit: %d", err);
        free(pot);
        return err;
    }

    adc_oneshot_chan_cfg_t chan_config = {
        .bitwidth = config->bitwidth,
        .atten = config->atten,
    };
    err = adc_oneshot_config_channel(adc_handle, config->channel, &chan_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC channel: %d", err);
        adc_oneshot_del_unit(adc_handle);
        free(pot);
        return err;
    }

    pot->adc_handle = adc_handle;
    pot->channel = ADC_CHANNEL_7;
    ESP_LOGI(TAG, "channel set to %d at %p", pot->channel, (void*)&pot->channel);
    pot->avg_samples = config->avg_samples;
    pot->max_voltage = config->max_voltage;
    pot->bit_max = bitwidth_to_max(config->bitwidth);
    pot->accumulator = 0.0f;
    pot->raw_min = 200.0f;
    pot->raw_max = 3000.0f;
    pot->has_reading = false;

    *out_handle = pot;
    return ESP_OK;
}

esp_err_t pot_deinit(pot_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = adc_oneshot_del_unit(handle->adc_handle);
    free(handle);
    return err;
}

esp_err_t pot_update(pot_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int adc_sample = 0;
    float adc_accumulator = 0.0f;

    for (uint32_t i = 0; i < handle->avg_samples; i++) {
        esp_err_t err = adc_oneshot_read(handle->adc_handle, handle->channel, &adc_sample);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "ADC read failed: %d", err);
            return err;
        }
        adc_accumulator += (float)adc_sample;
    }
    adc_accumulator /= (float)handle->avg_samples;

    handle->accumulator = adc_accumulator;
    handle->has_reading = true;

    if (adc_accumulator > handle->raw_max) {
        handle->raw_max = adc_accumulator;
    }
    if (adc_accumulator < handle->raw_min) {
        handle->raw_min = adc_accumulator;
    }

    return ESP_OK;
}

esp_err_t pot_get_raw(pot_handle_t handle, float *out_raw)
{
    if (handle == NULL || out_raw == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!handle->has_reading) {
        return ESP_ERR_INVALID_STATE;
    }
    *out_raw = handle->accumulator;
    return ESP_OK;
}

esp_err_t pot_get_percentage(pot_handle_t handle, float *out_percentage)
{
    if (handle == NULL || out_percentage == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!handle->has_reading) {
        return ESP_ERR_INVALID_STATE;
    }

    float range = handle->raw_max - handle->raw_min;
    if (range <= 0.0f) {
        // Not enough variation observed yet (e.g. only one reading so far).
        *out_percentage = 0.0f;
        return ESP_OK;
    }

    float percentage = ((handle->accumulator - (handle->raw_min)) / range) * 100.0f;
    if (percentage < 0.0f) percentage = 0.0f;
    if (percentage > 100.0f) percentage = 100.0f;

    *out_percentage = percentage;
    return ESP_OK;
}

esp_err_t pot_get_voltage(pot_handle_t handle, float *out_voltage)
{
    if (handle == NULL || out_voltage == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!handle->has_reading) {
        return ESP_ERR_INVALID_STATE;
    }

    *out_voltage = handle->accumulator * (handle->max_voltage / (handle->bit_max + 1.0f));
    return ESP_OK;
}
