/*
 * encoder.c — Quadrature encoder driver built on the ESP-IDF PCNT peripheral.
 */
#include <stdlib.h>
#include <string.h>
#include "encoder.h"
#include "driver/pulse_cnt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

static const char *TAG = "encoder";

struct encoder_dev_t {
    pcnt_unit_handle_t    unit;
    pcnt_channel_handle_t chan_a;
    pcnt_channel_handle_t chan_b;

    /* Optional event machinery — only allocated if watch points are used. */
    QueueHandle_t      evt_queue;
    TaskHandle_t        evt_task;
    encoder_event_cb_t  evt_cb;
    void               *evt_ctx;
};

static bool pcnt_on_reach_isr(pcnt_unit_handle_t unit,
                               const pcnt_watch_event_data_t *edata,
                               void *user_ctx)
{
    BaseType_t high_task_wakeup = pdFALSE;
    struct encoder_dev_t *dev = (struct encoder_dev_t *)user_ctx;
    xQueueSendFromISR(dev->evt_queue, &(edata->watch_point_value), &high_task_wakeup);
    return (high_task_wakeup == pdTRUE);
}

/* Library-owned task: blocks on the queue so user callbacks never run in
 * ISR context. */
static void encoder_event_task(void *arg)
{
    struct encoder_dev_t *dev = (struct encoder_dev_t *)arg;
    int watch_point_value;

    while (1) {
        if (xQueueReceive(dev->evt_queue, &watch_point_value, portMAX_DELAY)) {
            if (dev->evt_cb) {
                dev->evt_cb(watch_point_value, dev->evt_ctx);
            }
        }
    }
}

esp_err_t encoder_init(const encoder_config_t *config, encoder_handle_t *out_handle, encoder_event_cb_t cb)
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
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num  = config->gpio_a,
        .level_gpio_num = config->gpio_b,
    };
    ret = pcnt_new_channel(dev->unit, &chan_a_config, &dev->chan_a);
    if (ret != ESP_OK) {
        goto fail_channels;
    }

    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num  = config->gpio_b,
        .level_gpio_num = config->gpio_a,
    };
    ret = pcnt_new_channel(dev->unit, &chan_b_config, &dev->chan_b);
    if (ret != ESP_OK) {
        goto fail_channels;
    }

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(dev->chan_a,
                        PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(dev->chan_a,
                        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(dev->chan_b,
                        PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(dev->chan_b,
                        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // set watchpoints
    int watch_points[] = {
        config->pcnt_low_limit, -50, 0,
        50, config->pcnt_high_limit
    };
    int num_points = 5;

    for (size_t i = 0; i < num_points; i++) {
        ret = pcnt_unit_add_watch_point(dev->unit, watch_points[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to register watch point: %d - %s", watch_points[i], esp_err_to_name(ret));
            return ret;
        }
    }

    dev->evt_cb  = cb;
    dev->evt_queue = xQueueCreate(10, sizeof(int));
    if (!dev->evt_queue) {
        return ESP_ERR_NO_MEM;
    }

    pcnt_event_callbacks_t cbs = {
        .on_reach = pcnt_on_reach_isr,
    };
    ret = pcnt_unit_register_event_callbacks(dev->unit, &cbs, dev);
    if (ret != ESP_OK) {
        vQueueDelete(dev->evt_queue);
        dev->evt_queue = NULL;
        ESP_LOGE(TAG, "Failed to register event callbacks: %s", esp_err_to_name(ret));
        return ret;
    }

    BaseType_t task_ret = xTaskCreate(encoder_event_task, "encoder_evt",
                                       2048, dev, tskIDLE_PRIORITY + 2,
                                       &dev->evt_task);
    if (task_ret != pdPASS) {
        vQueueDelete(dev->evt_queue);
        dev->evt_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

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

esp_err_t encoder_register_watchpoints(encoder_handle_t handle,
                                        const int *watch_points,
                                        size_t num_points,
                                        encoder_event_cb_t cb,
                                        void *user_ctx)
{
    if (!handle || (num_points > 0 && !watch_points)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (num_points == 0) {
        return ESP_OK; /* nothing to do */
    }

    for (size_t i = 0; i < num_points; i++) {
        esp_err_t ret = pcnt_unit_add_watch_point(handle->unit, watch_points[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to register watch point: %d - %s", watch_points[i], esp_err_to_name(ret));
            return ret;
        }
    }

    handle->evt_cb  = cb;
    handle->evt_ctx = user_ctx;
    handle->evt_queue = xQueueCreate(10, sizeof(int));
    if (!handle->evt_queue) {
        return ESP_ERR_NO_MEM;
    }

    pcnt_event_callbacks_t cbs = {
        .on_reach = pcnt_on_reach_isr,
    };
    esp_err_t ret = pcnt_unit_register_event_callbacks(handle->unit, &cbs, handle);
    if (ret != ESP_OK) {
        vQueueDelete(handle->evt_queue);
        handle->evt_queue = NULL;
        ESP_LOGE(TAG, "Failed to register event callbacks: %s", esp_err_to_name(ret));
        return ret;
    }

    BaseType_t task_ret = xTaskCreate(encoder_event_task, "encoder_evt",
                                       2048, handle, tskIDLE_PRIORITY + 2,
                                       &handle->evt_task);
    if (task_ret != pdPASS) {
        vQueueDelete(handle->evt_queue);
        handle->evt_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
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

    if (handle->evt_task) {
        vTaskDelete(handle->evt_task);
    }
    if (handle->evt_queue) {
        vQueueDelete(handle->evt_queue);
    }

    pcnt_unit_disable(handle->unit);
    pcnt_del_channel(handle->chan_a);
    pcnt_del_channel(handle->chan_b);
    pcnt_del_unit(handle->unit);

    free(handle);
    return ESP_OK;
}
