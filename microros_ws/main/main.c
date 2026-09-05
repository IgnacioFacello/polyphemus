#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include <esp_adc/adc_oneshot.h>
#include "sdkconfig.h"

#include "driver/potentiometer.h"
#include "driver/as5600.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/twist.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn)                                                                      \
    {                                                                                    \
        rcl_ret_t temp_rc = fn;                                                          \
        if ((temp_rc != RCL_RET_OK))                                                     \
        {                                                                                \
            printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
            vTaskDelete(NULL);                                                           \
        }                                                                                \
    }
#define RCSOFTCHECK(fn)                                                                    \
    {                                                                                      \
        rcl_ret_t temp_rc = fn;                                                            \
        if ((temp_rc != RCL_RET_OK))                                                       \
        {                                                                                  \
            printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
        }                                                                                  \
    }
#define MICRO_ROS_APP_STACK 16000
#define MICRO_ROS_APP_TASK_PRIO 5

#ifndef MICROROS_NAMESPACE
#define MICROROS_NAMESPACE ""
#endif

#ifndef DOMAIN_ID
#define DOMAIN_ID 0
#endif

#define TIMER_PERIOD_MS 100

#define PI 3.14

static const char *TAG = "micro_ros";

// Global Handles
static rcl_publisher_t poten_publisher;
std_msgs__msg__Float32 poten_msg;

static rcl_publisher_t encoder_publisher;
std_msgs__msg__Float32 encoder_msg;

static encoder_handle_t encoder_h;

static pot_handle_t pot_handle;
static pot_config_t pot_cfg = {
    .unit = ADC_UNIT_1,
    .channel = ADC_CHANNEL_7, // GPIO35
    .bitwidth = ADC_BITWIDTH_12,
    .atten = ADC_ATTEN_DB_6,
    .max_voltage = 2.2f,
    .avg_samples = 25,
};


void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    float poten_percent = 0.0f;
    float poten_rad = 0.0f;
    uint16_t encoder_count = 0;
    uint16_t encoder_raw = 0;
    float encoder_rad = 0.0f;

    uint8_t enc_status = 0x00;

    if (pot_handle != NULL)
    {
        pot_update(pot_handle);
        pot_get_percentage(pot_handle, &poten_percent);
        poten_rad = (poten_percent/100.0) * 2*PI;
        poten_msg.data = poten_rad;
        RCSOFTCHECK(rcl_publish(&poten_publisher, &poten_msg, NULL));
    }

    if (encoder_h != NULL) {
        as5600_get_status(encoder_h, &enc_status);
        if (enc_status & 0x20) {
            as5600_get_angle(encoder_h, &encoder_count);
            as5600_get_raw_angle(encoder_h, &encoder_raw);
            ESP_LOGI(TAG, "Encoder count: %d, raw: %d", encoder_count, encoder_raw);
            encoder_rad = ((float)encoder_count/4095) * 2*PI;
            encoder_msg.data = encoder_rad;
            RCSOFTCHECK(rcl_publish(&encoder_publisher, &encoder_msg, NULL));
        } else {
            as5600_check_status(enc_status);
        }
    }

    //ESP_LOGI(TAG, "Potentiometer: %.2f, Encoder: %.2f", poten_rad, encoder_rad);
}

/* ── Tarea micro-ROS ────────────────────────────────────────── */
void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_ret_t rc;
    int retry_count = 0;
    const int MAX_RETRIES = 5;
    const int RETRY_DELAY_MS = 2000;

    // Give network time to stabilize after WiFi connects
    ESP_LOGI(TAG, "Waiting for network to stabilize...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rc = rcl_init_options_init(&init_options, allocator);
    if (rc != RCL_RET_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize init_options: %d", rc);
        vTaskDelete(NULL);
        return;
    }

    // Setear el DOMAIN_ID
    rc = rcl_init_options_set_domain_id(&init_options, DOMAIN_ID);
    if (rc != RCL_RET_OK)
    {
        ESP_LOGE(TAG, "Failed to set domain id: %d", rc);
        vTaskDelete(NULL);
        return;
    }

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    rc = rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP,
                                          CONFIG_MICRO_ROS_AGENT_PORT,
                                          rmw_options);
    if (rc != RCL_RET_OK)
    {
        ESP_LOGE(TAG, "Failed to set UDP address: %d", rc);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "UDP configured: %s:%s", CONFIG_MICRO_ROS_AGENT_IP,
             CONFIG_MICRO_ROS_AGENT_PORT);
#endif

    // Retry loop for rclc_support_init
    while (retry_count < MAX_RETRIES)
    {
        ESP_LOGI(TAG, "Attempting micro-ROS support init (attempt %d/%d)...",
                 retry_count + 1, MAX_RETRIES);
        rc = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

        if (rc == RCL_RET_OK)
        {
            ESP_LOGI(TAG, "Micro-ROS support initialized successfully");
            break;
        }
        else
        {
            ESP_LOGW(TAG, "Failed status on rclc_support_init: %d. %s", rc,
                     retry_count < MAX_RETRIES - 1 ? "Retrying..." : "Max retries reached. Aborting.");

            if (retry_count < MAX_RETRIES - 1)
            {
                vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
                retry_count++;
            }
            else
            {
                vTaskDelete(NULL);
                return;
            }
        }
    }

    rcl_node_t node = rcl_get_zero_initialized_node();
    rc = rclc_node_init_default(&node, "microros_node", MICROROS_NAMESPACE, &support);
    if (rc != RCL_RET_OK)
    {
        ESP_LOGE(TAG, "Failed to init node: %d", rc);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Node created successfully");

    // Inicialización del publicador de posición
    rc = rclc_publisher_init_default(
        &poten_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "potentiometer"
    );

    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to init potentiometer publisher: %d", rc);
        vTaskDelete(NULL);
        return;
    }
    else { ESP_LOGI(TAG, "Potentiometer publisher initialized"); }

    rc = rclc_publisher_init_default(
        &encoder_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "encoder"
    );
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to init encoder publisher: %d", rc);
        vTaskDelete(NULL);
        return; }
    else { ESP_LOGI(TAG, "Encoder publisher initialized"); }

    // Inicialización del Timer
    rcl_timer_t timer = rcl_get_zero_initialized_timer();
    rc = rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS(TIMER_PERIOD_MS),
        timer_callback,
        true);
    if (rc != RCL_RET_OK)
    {
        ESP_LOGE(TAG, "Failed to init timer: %d", rc);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Timer initialized");

    // Executor
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    rc = rclc_executor_init(&executor, &support.context, 3, &allocator);
    if (rc != RCL_RET_OK)
    {
        ESP_LOGE(TAG, "Failed to init executor: %d", rc);
        vTaskDelete(NULL);
        return;
    }

    rc = rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(1000));
    if (rc != RCL_RET_OK)
    {
        ESP_LOGW(TAG, "Failed to set executor timeout: %d (continuing)", rc);
    }

    rc = rclc_executor_add_timer(&executor, &timer);
    if (rc != RCL_RET_OK)
    {
        ESP_LOGE(TAG, "Failed to add timer to executor: %d", rc);
        vTaskDelete(NULL);
        return;
    }

    if (rc != RCL_RET_OK)
    {
        ESP_LOGE(TAG, "Failed to add service to executor: %d", rc);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Executor configured successfully. Starting main loop...");

    pot_init(&pot_cfg, &pot_handle);
    as5600_init(&encoder_h);
    set_start_pos(encoder_h, 3569);
    //set_stop_pos(encoder_h, 4095);
    //set_max_angle(encoder_h, 4095);

    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(10000);
    }

    pot_deinit(pot_handle);
    as5600_deinit(encoder_h);

    RCCHECK(rcl_publisher_fini(&poten_publisher, &node));
    RCCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);
}

void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    xTaskCreate(micro_ros_task, "micro_ros_task",
                MICRO_ROS_APP_STACK, NULL, MICRO_ROS_APP_TASK_PRIO, NULL);

}
