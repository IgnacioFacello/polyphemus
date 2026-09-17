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
#include "sdkconfig.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32_multi_array.h>
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

// Tamaño del array de entrada: 3 componentes de vector + 3 ángulos.
// Si más adelante necesitás más datos (ej. un cuarto ángulo, o un segundo vector),
// solo cambiá este número.
#define ROTATION_INPUT_SIZE 7
#define ROTATION_OUTPUT_SIZE 3 // vector resultado (x, y, z)

static const char *TAG = "micro_ros";

// Publisher: publica el vector ya rotado
static rcl_publisher_t vector_rotado_pub;
static std_msgs__msg__Float32MultiArray vector_rotado_msg;

// Subscriber: recibe vector + ángulos a aplicar
static rcl_subscription_t rotation_sub;
static std_msgs__msg__Float32MultiArray rotation_msg;

/**
 * Se ejecuta automáticamente cada vez que llega un mensaje nuevo al tópico
 * "rotation_input". El middleware ya dejó los datos recibidos escritos
 * dentro de rotation_msg.data.data antes de llamar a esta función.
 */
void rotation_callback(const void *msgin)
{
    const std_msgs__msg__Float32MultiArray *msg =
        (const std_msgs__msg__Float32MultiArray *)msgin;

    if (msg->data.size < ROTATION_INPUT_SIZE)
    {
        ESP_LOGW(TAG, "Mensaje recibido con tamaño inesperado: %d (esperado %d)",
                 (int)msg->data.size, ROTATION_INPUT_SIZE);
        return;
    }

    float x = msg->data.data[0];
    float y = msg->data.data[1];
    float z = msg->data.data[2];
    float angulo_x = msg->data.data[3];
    float angulo_y = msg->data.data[4];
    float angulo_z = msg->data.data[5];
    int pasos = msg->data.data[6]

    ESP_LOGI(TAG, "Recibido vector: (%.3f, %.3f, %.3f) angulos: (%.3f, %.3f, %.3f) pasos: %d" , 
             x, y, z, angulo_x, angulo_y, angulo_z, pasos);

   // ACA poner codigo de rotacion

    vector_rotado_msg.data.data[0] = x;
    vector_rotado_msg.data.data[1] = y;
    vector_rotado_msg.data.data[2] = z;

    RCSOFTCHECK(rcl_publish(&vector_rotado_pub, &vector_rotado_msg, NULL));
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

    // buffer para datos recibidos
    rotation_msg.data.data = (float *)malloc(ROTATION_INPUT_SIZE * sizeof(float));
    rotation_msg.data.size = 0;
    rotation_msg.data.capacity = ROTATION_INPUT_SIZE;

    rc = rclc_subscription_init_default(
        &rotation_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "rotation_input");
    if (rc != RCL_RET_OK)
    {
        ESP_LOGE(TAG, "Failed to init rotation_sub: %d", rc);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Subscriber initialized");

    //  buffer del mensaje que vamos a publicar
    vector_rotado_msg.data.data = (float *)malloc(ROTATION_OUTPUT_SIZE * sizeof(float));
    vector_rotado_msg.data.size = ROTATION_OUTPUT_SIZE;
    vector_rotado_msg.data.capacity = ROTATION_OUTPUT_SIZE;

    rc = rclc_publisher_init_default(
        &vector_rotado_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "vector_rotado");
    if (rc != RCL_RET_OK)
    {
        ESP_LOGE(TAG, "Failed to init vector_rotado publisher: %d", rc);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Publisher initialized");

    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    rc = rclc_executor_init(&executor, &support.context, 1, &allocator);
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

    rc = rclc_executor_add_subscription(
        &executor,
        &rotation_sub,
        &rotation_msg,
        &rotation_callback,
        ON_NEW_DATA);
    if (rc != RCL_RET_OK)
    {
        ESP_LOGE(TAG, "Failed to add subscription to executor: %d", rc);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Executor configured successfully. Starting main loop...");

    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(10000);
    }

    RCCHECK(rcl_subscription_fini(&rotation_sub, &node));
    RCCHECK(rcl_publisher_fini(&vector_rotado_pub, &node));
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