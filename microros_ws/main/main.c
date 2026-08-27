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

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int64.h>
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

// #define ENCODER_GPIO_A 32
// #define ENCODER_GPIO_B 33
// #define ENCODER_PPR 600
#define TIMER_PERIOD_MS 100 // Reducido de 400ms
#define AVG_SAMPLES 75      // Numero de muestras por mensaje

#define ADC_PIN ADC_CHANNEL_7        // Channel 7 - Check ESP32 Pinout for the GPIO Number
#define ADC_UNIT ADC_UNIT_1          // ADC1
#define ADC_BITWIDTH ADC_BITWIDTH_12 // 12-bit resolution (0-4095)
#define ADC_ATTEN ADC_ATTEN_DB_12    // ~3.3V full-scale voltage

static const char *TAG = "micro_ros";

static rcl_publisher_t position_publisher;
std_msgs__msg__Float32 position_msg;

static rcl_publisher_t voltage_publisher;
std_msgs__msg__Float32 voltage_msg;

static adc_oneshot_unit_handle_t adc_handle = NULL;
static const float MAX_VOLTAGE = 3.3f; // Voltaje máximo según ATTEN_DB_12
float ADC_MAX_VALUE = 1; // Resolución 12-bit
float ADC_MIN_VALUE = 4096; // Resolución 12-bit


void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    if (timer == NULL || adc_handle == NULL)
        return;

    int adc_sample = 0;
    float adc_accumulator = 0.0;
    (void)last_call_time;

    for (int i=0; i<AVG_SAMPLES; i++) {
        esp_err_t adc_err = adc_oneshot_read(adc_handle, ADC_PIN, &adc_sample);
        if (adc_err != ESP_OK)
        {
            ESP_LOGW(TAG, "ADC read failed: %d", adc_err);
            return;
        }
        adc_accumulator += (float)adc_sample;
    }
    adc_accumulator = adc_accumulator/(float)AVG_SAMPLES;

    if(ADC_MAX_VALUE < adc_accumulator){
        ADC_MAX_VALUE = adc_accumulator;
    }
    if(ADC_MIN_VALUE > adc_accumulator){
        ADC_MIN_VALUE = adc_accumulator;
    }

    // 2. Calcular posición en porcentaje (0-100)
    float posicion_porcentaje = ((adc_accumulator - ADC_MIN_VALUE) / ADC_MAX_VALUE) * 100.0f;

    // 3. Calcular voltaje (0-3.3V)
    float voltaje = ((adc_accumulator - ADC_MIN_VALUE) / ADC_MAX_VALUE) * MAX_VOLTAGE;

    // 4. Publicar posición
    position_msg.data = posicion_porcentaje;
    RCSOFTCHECK(rcl_publish(&position_publisher, &position_msg, NULL));

    // 5. Publicar voltaje
    voltage_msg.data = voltaje;
    RCSOFTCHECK(rcl_publish(&voltage_publisher, &voltage_msg, NULL));

    ESP_LOGI(TAG, "min: %.2f | mean ADC: %.2f | max: %.2f | Posición: %.2f%% | Voltaje: %.2fV",
             ADC_MIN_VALUE, adc_accumulator, ADC_MAX_VALUE, posicion_porcentaje, voltaje);
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

    // Inicialización del publicador
    rc = rclc_publisher_init_default(
        &position_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "percentage_position");
    if (rc != RCL_RET_OK)
    {
        ESP_LOGE(TAG, "Failed to init position_publisher: %d", rc);
        vTaskDelete(NULL);
        return;
    }

    rc = rclc_publisher_init_default(
        &voltage_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "voltage");
    if (rc != RCL_RET_OK)
    {
        ESP_LOGE(TAG, "Failed to init voltage_publisher: %d", rc);
        vTaskDelete(NULL);
        return;
    }

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

    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(10000);
    }

    RCCHECK(rcl_publisher_fini(&position_publisher, &node));
    RCCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);
}

void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    // Configure ADC
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH,
        .atten = ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_PIN, &config));

    xTaskCreate(micro_ros_task, "micro_ros_task",
                MICRO_ROS_APP_STACK, NULL, MICRO_ROS_APP_TASK_PRIO, NULL);
}
