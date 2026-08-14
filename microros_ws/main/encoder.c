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

    int              pcnt_high_limit;
    int              pcnt_low_limit;

    /* Accumulated offset from wrap events at the high/low limits, added to
     * the raw hardware count to produce a continuous, unwrapped total. Only
     * ever touched from within the on_reach ISR (write) and
     * encoder_get_total_count()/encoder_clear_count() (read/reset), always
     * under `spinlock`. */
    volatile int64_t accum;
    portMUX_TYPE      spinlock;
};

/* Runs in ISR context whenever the raw count reaches pcnt_high_limit or
 * pcnt_low_limit. At that moment the PCNT hardware wraps the raw count back
 * near zero, so we record the offset needed to keep a continuous total. */
static bool IRAM_ATTR pcnt_on_reach_isr(pcnt_unit_handle_t unit,
                                         const pcnt_watch_event_data_t *edata,
                                         void *user_ctx)
{
    struct encoder_dev_t *dev = (struct encoder_dev_t *)user_ctx;

    portENTER_CRITICAL_ISR(&dev->spinlock);
    if (edata->watch_point_value == dev->pcnt_high_limit) {
        dev->accum += dev->pcnt_high_limit;
    } else if (edata->watch_point_value == dev->pcnt_low_limit) {
        dev->accum += dev->pcnt_low_limit; /* low_limit is negative */
    }
    portEXIT_CRITICAL_ISR(&dev->spinlock);

    return false; /* no higher-priority task woken */
}

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

    dev->pcnt_high_limit = config->pcnt_high_limit;
    dev->pcnt_low_limit  = config->pcnt_low_limit;
    dev->accum = 0;
    dev->spinlock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;

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

    // Watch points para detectar wraparound — deben registrarse ANTES de
    // pcnt_unit_enable(), el driver PCNT solo los acepta con la unidad en
    // estado "init".
    ret = pcnt_unit_add_watch_point(dev->unit, config->pcnt_high_limit);
    if (ret != ESP_OK) {
        goto fail_channels;
    }
    ret = pcnt_unit_add_watch_point(dev->unit, config->pcnt_low_limit);
    if (ret != ESP_OK) {
        goto fail_channels;
    }
    pcnt_event_callbacks_t cbs = {
        .on_reach = pcnt_on_reach_isr,
    };
    ret = pcnt_unit_register_event_callbacks(dev->unit, &cbs, dev);
    if (ret != ESP_OK) {
        goto fail_channels;
    }

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

esp_err_t encoder_get_total_count(encoder_handle_t handle, int64_t *total_count)
{
    if (!handle || !total_count) {
        return ESP_ERR_INVALID_ARG;
    }

    int raw = 0;
    esp_err_t ret = pcnt_unit_get_count(handle->unit, &raw);
    if (ret != ESP_OK) {
        return ret;
    }

    // Nota: hay una ventana teórica muy pequeña entre leer `raw` y tomar el
    // spinlock donde podría ocurrir un wrap; en la práctica, a las
    // frecuencias típicas de un encoder esto es despreciable. Si se necesita
    // exactitud perfecta, leer accum y raw dentro de la misma sección
    // crítica requeriría acceder al registro del PCNT directamente.
    portENTER_CRITICAL(&handle->spinlock);
    *total_count = handle->accum + raw;
    portEXIT_CRITICAL(&handle->spinlock);

    return ESP_OK;
}

esp_err_t encoder_clear_count(encoder_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&handle->spinlock);
    handle->accum = 0;
    portEXIT_CRITICAL(&handle->spinlock);

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
