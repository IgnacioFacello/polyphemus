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
#include "encoder.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int64.h>
#include <geometry_msgs/msg/twist.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>


#include <custom_interfaces/srv/motor.h>
#include <custom_interfaces/msg/motor_rpm.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define MICRO_ROS_APP_STACK 16000
#define MICRO_ROS_APP_TASK_PRIO 5

#ifndef MICROROS_NAMESPACE
    #define MICROROS_NAMESPACE ""
#endif

#ifndef DOMAIN_ID
    #define DOMAIN_ID 0
#endif

#define ENCODER_GPIO_A 32
#define ENCODER_GPIO_B 33
#define ENCODER_PPR 600
#define TIMER_PERIOD_MS 100

static const char *TAG = "micro_ros";

static rcl_publisher_t angle_publisher;
std_msgs__msg__Int64 angle_msg;

static rcl_publisher_t rpm_publisher;
std_msgs__msg__Float32 rpm_msg;

encoder_config_t enc_cfg = ENCODER_DEFAULT_CONFIG(ENCODER_GPIO_A, ENCODER_GPIO_B, ENCODER_PPR*4);
encoder_handle_t enc;
static int64_t previous_count = 0;
/* ── Callbacks micro-ROS ────────────────────────────────────── */


// Función para calcular RPM
float calculate_rpm(int64_t current, int64_t previous)
{
    float delta_ticks = (float)(current - previous);     // Diferencia en ticks
    float time_sec = (float)TIMER_PERIOD_MS / 1000.0;  // Tiempo transcurrido en segundos

    // 1. delta_angle / 360.0 -> Convierte los grados a VUELTAS
    // 2. / time_sec         ->  cantidad de vueltas por SEGUNDO
    // 3. * 60.0             -> Multiplica por 60 para pasarlo a MINUTOS (RPM)
    float rpm = (delta_ticks / (float)(ENCODER_PPR*4)) / time_sec * 60.0;

    return rpm;
}

float calculate_angle(int count){
    float deg_per_pulse = 360.0 / ((float)ENCODER_PPR*4);
    return (float)count * deg_per_pulse;
}

// Ahora en el timer callback
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    int count = 0;
    int64_t total_count = 0;
    (void) last_call_time;
    if (timer == NULL)
        return;

    RCSOFTCHECK(encoder_get_count(enc, &count));
    RCSOFTCHECK(encoder_get_total_count(enc, &total_count));

    // Calcular RPM
    float angle = calculate_angle(count);
    float rpm = calculate_rpm(total_count, previous_count);

    // Log para ver ambos datos
    ESP_LOGI(TAG, "Position: %.2f deg | RPM: %.2f rpm", angle, rpm);

    // Publicar
    angle_msg.data = total_count;
    RCSOFTCHECK(rcl_publish(&angle_publisher, &angle_msg, NULL));

    rpm_msg.data = rpm;
    RCSOFTCHECK(rcl_publish(&rpm_publisher, &rpm_msg, NULL));

    // Guardar para la próxima lectura
    previous_count = total_count;
}

/* ── Tarea micro-ROS ────────────────────────────────────────── */
void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t  support;
    rcl_ret_t rc;
    int retry_count = 0;
    const int MAX_RETRIES = 5;
    const int RETRY_DELAY_MS = 2000;

    // Give network time to stabilize after WiFi connects
    ESP_LOGI(TAG, "Waiting for network to stabilize...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rc = rcl_init_options_init(&init_options, allocator);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to initialize init_options: %d", rc);
        vTaskDelete(NULL);
        return;
    }

    // Setear el DOMAIN_ID
    rc = rcl_init_options_set_domain_id(&init_options, DOMAIN_ID);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to set domain id: %d", rc);
        vTaskDelete(NULL);
        return;
    }

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    rc = rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP,
                                         CONFIG_MICRO_ROS_AGENT_PORT,
                                         rmw_options);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to set UDP address: %d", rc);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "UDP configured: %s:%s", CONFIG_MICRO_ROS_AGENT_IP,
             CONFIG_MICRO_ROS_AGENT_PORT);
#endif

    // Retry loop for rclc_support_init
    while (retry_count < MAX_RETRIES) {
        ESP_LOGI(TAG, "Attempting micro-ROS support init (attempt %d/%d)...",
                 retry_count + 1, MAX_RETRIES);
        rc = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

        if (rc == RCL_RET_OK) {
            ESP_LOGI(TAG, "Micro-ROS support initialized successfully");
            break;
        } else {
            ESP_LOGW(TAG, "Failed status on rclc_support_init: %d. %s", rc,
                    retry_count < MAX_RETRIES - 1 ? "Retrying..." : "Max retries reached. Aborting.");


            if (retry_count < MAX_RETRIES - 1) {
                vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
                retry_count++;
            } else {
                vTaskDelete(NULL);
                return;
            }
        }
    }

    rcl_node_t node = rcl_get_zero_initialized_node();
    rc = rclc_node_init_default(&node, "microros_node", MICROROS_NAMESPACE, &support);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to init node: %d", rc);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Node created successfully");

    // Inicialización del publicador
    rc = rclc_publisher_init_default(
        &angle_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
        "angle_data");
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to init angle_publisher: %d", rc);
        vTaskDelete(NULL);
        return;
    }

    rc = rclc_publisher_init_default(
        &rpm_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "rpm_data");
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to init rpm_publisher: %d", rc);
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
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to init timer: %d", rc);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Timer initialized");

    // Executor
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    rc = rclc_executor_init(&executor, &support.context, 3, &allocator);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to init executor: %d", rc);
        vTaskDelete(NULL);
        return;
    }

    rc = rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(1000));
    if (rc != RCL_RET_OK) {
        ESP_LOGW(TAG, "Failed to set executor timeout: %d (continuing)", rc);
    }

    rc = rclc_executor_add_timer(&executor, &timer);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to add timer to executor: %d", rc);
        vTaskDelete(NULL);
        return;
    }

    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to add service to executor: %d", rc);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Executor configured successfully. Starting main loop...");

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(10000);
    }

    RCCHECK(rcl_publisher_fini(&angle_publisher, &node));
    RCCHECK(rcl_node_fini(&node));
    encoder_deinit(enc);
    vTaskDelete(NULL);
}


void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    ESP_LOGI("main", "pins  : %d, %d", enc_cfg.gpio_a, enc_cfg.gpio_b);
    ESP_LOGI("main", "limits: [%d, %d]", enc_cfg.pcnt_low_limit, enc_cfg.pcnt_high_limit);
    ESP_ERROR_CHECK(gpio_set_pull_mode(ENCODER_GPIO_A, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(gpio_set_pull_mode(ENCODER_GPIO_B, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(encoder_init(&enc_cfg, &enc));

    xTaskCreate(micro_ros_task, "micro_ros_task",
                MICRO_ROS_APP_STACK, NULL, MICRO_ROS_APP_TASK_PRIO, NULL);
}
