/*
 * encoder.c — Quadrature encoder driver built on the ESP-IDF PCNT peripheral.
 */
#include <stdlib.h>
#include "encoder.h"
#include "driver/pulse_cnt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

static const char *TAG = "encoder";

struct encoder_dev_t {
    pcnt_unit_handle_t    unit;
    pcnt_channel_handle_t chan_one;
    pcnt_channel_handle_t chan_two;
};

esp_err_t encoder_init(const encoder_config_t *config, encoder_handle_t *out_handle)
{
    // Verifica correctitud de los argumentos
    if (!config || !out_handle) {
        return ESP_ERR_INVALID_ARG;
    }

    struct encoder_dev_t *dev = calloc(1, sizeof(*dev));
    if (!dev) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = ESP_OK;

    // Creación de la unidad
    pcnt_unit_config_t unit_config = {
        .high_limit = config->pcnt_high_limit,
        .low_limit  = config->pcnt_low_limit,
    };
    ret = pcnt_new_unit(&unit_config, &dev->unit);
    if (ret != ESP_OK) {
        goto fail_unit;
    }

    // Configuración del glitch filter (resuelve bounceback?)
    if (config->glitch_filter_ns > 0) {
        pcnt_glitch_filter_config_t filter_config = {
            .max_glitch_ns = config->glitch_filter_ns,
        };
        ret = pcnt_unit_set_glitch_filter(dev->unit, &filter_config);
        if (ret != ESP_OK) {
            goto fail_channels;
        }
    }

    // Creación de los canales
    pcnt_chan_config_t chan_one_config = {
        .edge_gpio_num  = config->gpio_a,
        .level_gpio_num = config->gpio_b,
    };
    ret = pcnt_new_channel(dev->unit, &chan_one_config, &dev->chan_one);
    if (ret != ESP_OK) {
        goto fail_channels;
    }

    pcnt_chan_config_t chan_two_config = {
        .edge_gpio_num  = config->gpio_b,
        .level_gpio_num = config->gpio_a,
    };
    ret = pcnt_new_channel(dev->unit, &chan_two_config, &dev->chan_two);
    if (ret != ESP_OK) {
        goto fail_channels;
    }

    // Configuración de los canales
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(dev->chan_one,
                        PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(dev->chan_one,
                        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(dev->chan_two,
                        PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(dev->chan_two,
                        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // Activación de la unidad
    ESP_ERROR_CHECK(pcnt_unit_enable(dev->unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(dev->unit));
    ESP_ERROR_CHECK(pcnt_unit_start(dev->unit));

    *out_handle = dev;
    return ESP_OK;

fail_channels:
    pcnt_del_unit(dev->unit);
fail_unit:
    free(dev);
    ESP_LOGE(TAG, "encoder_init failed: %s", esp_err_to_name(ret));
    return ret;
}

esp_err_t encoder_get_count(encoder_handle_t handle, int *count)
{
    if (!handle || !count) {
        return ESP_ERR_INVALID_ARG;
    }
    return pcnt_unit_get_count(handle->unit, count);
}

esp_err_t encoder_clear_count(encoder_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    return pcnt_unit_clear_count(handle->unit);
}

esp_err_t encoder_deinit(encoder_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    pcnt_unit_disable(handle->unit);
    pcnt_del_channel(handle->chan_one);
    pcnt_del_channel(handle->chan_two);
    pcnt_del_unit(handle->unit);

    free(handle);
    return ESP_OK;
}
