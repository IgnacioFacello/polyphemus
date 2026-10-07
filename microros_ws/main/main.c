#include <stdint.h>
#include <unistd.h>
#include <stdbool.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"


#include "esp_log.h"
#include "esp_err.h"
#include "sdkconfig.h"

#include "driver/mpu6050.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define MICRO_ROS_APP_STACK 16000
#define MICRO_ROS_APP_TASK_PRIO 5
#define TIMER_PERIOD_MS 100

#ifndef MICROROS_NAMESPACE
#define MICROROS_NAMESPACE ""
#endif

#ifndef DOMAIN_ID
#define DOMAIN_ID 0
#endif

static uint8_t data[2];
static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;

static const char *TAG = "micro_ros";
void leer_fifo_count(uint16_t *count)
{
    uint8_t buffer[2];

    esp_err_t err = mpu6050_register_read(dev_handle, MPU6050_FIFO_COUNT_H, buffer, 2);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "No se pudo leer FIFO_COUNT: %s", esp_err_to_name(err));
        
    }

    // buffer[0] tiene FIFO_COUNT_H
    // buffer[1] tiene FIFO_COUNT_L
    *count = ((uint16_t)buffer[0] << 8) | buffer[1];
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)timer;
    (void)last_call_time;
}

esp_err_t init_publishers(rcl_node_t *node)
{
    rcl_ret_t rc;

    // rc = rclc_publisher_init_default(
    //     &poten_publisher,
    //     node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(
    //         std_msgs,
    //         msg,
    //         Float32
    //     ),
    //     "potentiometer"
    // );

    // if (rc != RCL_RET_OK)
    // {
    //     ESP_LOGE(
    //         TAG,
    //         "Failed to init potentiometer publisher: %d",
    //         rc);

    //     return rc;
    // }
    return RCL_RET_OK;
}

void micro_ros_task(void *arg)
{
    (void)arg;

    rcl_allocator_t allocator =
        rcl_get_default_allocator();

    rclc_support_t support;
    rcl_ret_t rc;

    int retry_count = 0;

    const int MAX_RETRIES = 5;
    const int RETRY_DELAY_MS = 2000;

    ESP_LOGI(
        TAG,
        "Waiting for network to stabilize...");

    vTaskDelay(
        pdMS_TO_TICKS(2000));

    rcl_init_options_t init_options =
        rcl_get_zero_initialized_init_options();

    rc = rcl_init_options_init(
        &init_options,
        allocator);

    if (rc != RCL_RET_OK)
    {
        ESP_LOGE(
            TAG,
            "Failed to initialize init_options: %d",
            rc);

        vTaskDelete(NULL);
        return;
    }

    rc = rcl_init_options_set_domain_id(
        &init_options,
        DOMAIN_ID);

    if (rc != RCL_RET_OK)
    {
        ESP_LOGE(
            TAG,
            "Failed to set domain id: %d",
            rc);

        vTaskDelete(NULL);
        return;
    }

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE

    rmw_init_options_t *rmw_options =
        rcl_init_options_get_rmw_init_options(
            &init_options);

    rc = rmw_uros_options_set_udp_address(
        CONFIG_MICRO_ROS_AGENT_IP,
        CONFIG_MICRO_ROS_AGENT_PORT,
        rmw_options);

    if (rc != RCL_RET_OK)
    {
        ESP_LOGE(
            TAG,
            "Failed to set UDP address: %d",
            rc);

        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(
        TAG,
        "UDP configured: %s:%s",
        CONFIG_MICRO_ROS_AGENT_IP,
        CONFIG_MICRO_ROS_AGENT_PORT);

#endif

    while (retry_count < MAX_RETRIES)
    {
        ESP_LOGI(
            TAG,
            "Attempting micro-ROS support init (attempt %d/%d)...",
            retry_count + 1,
            MAX_RETRIES);

        rc = rclc_support_init_with_options(
            &support,
            0,
            NULL,
            &init_options,
            &allocator);

        if (rc == RCL_RET_OK)
        {
            ESP_LOGI(
                TAG,
                "Micro-ROS support initialized successfully");

            break;
        }

        ESP_LOGW(
            TAG,
            "Failed status on rclc_support_init: %d",
            rc);

        retry_count++;

        if (retry_count >= MAX_RETRIES)
        {
            vTaskDelete(NULL);
            return;
        }

        vTaskDelay(
            pdMS_TO_TICKS(RETRY_DELAY_MS));
    }

    rcl_node_t node =
        rcl_get_zero_initialized_node();

    rc = rclc_node_init_default(
        &node,
        "microros_node",
        MICROROS_NAMESPACE,
        &support);

    if (rc != RCL_RET_OK)
    {
        ESP_LOGE(
            TAG,
            "Failed to init node: %d",
            rc);

        vTaskDelete(NULL);
        return;
    }

    rc = init_publishers(
        &node);

    if (rc != RCL_RET_OK)
    {
        vTaskDelete(NULL);
        return;
    }

    rcl_timer_t timer =
        rcl_get_zero_initialized_timer();

    rc = rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS(TIMER_PERIOD_MS),
        timer_callback,
        true);

    if (rc != RCL_RET_OK)
    {
        ESP_LOGE(
            TAG,
            "Failed to init timer: %d",
            rc);

        vTaskDelete(NULL);
        return;
    }

    rclc_executor_t executor =
        rclc_executor_get_zero_initialized_executor();

    rc = rclc_executor_init(
        &executor,
        &support.context,
        1,
        &allocator);

    if (rc != RCL_RET_OK)
    {
        ESP_LOGE(
            TAG,
            "Failed to init executor: %d",
            rc);

        vTaskDelete(NULL);
        return;
    }

    rc = rclc_executor_add_timer(
        &executor,
        &timer);

    if (rc != RCL_RET_OK)
    {
        ESP_LOGE(
            TAG,
            "Failed to add timer to executor: %d",
            rc);

        vTaskDelete(NULL);
        return;
    }

    while (1)
    {
        rclc_executor_spin_some(
            &executor,
            RCL_MS_TO_NS(100));

        vTaskDelay(
            pdMS_TO_TICKS(10));
    }
}

/* ============================================================
 * APP MAIN
 * ============================================================ */
void app_main(void)
{
    // Interrupt Begin

    // Interrupt End

    i2c_master_init(&bus_handle, &dev_handle);
    if (dev_handle == NULL)
    {
        ESP_LOGE(TAG, "Error initializing I2C");
        return;
    }
    ESP_LOGI(TAG, "I2C initialized successfully");

    esp_err_t err = mpu6050_register_write_byte(dev_handle, MPU6050_PWR_MGMT_1_REG_ADDR, 0x01); // sacar de sleep + elegir reloj
    vTaskDelay(pdMS_TO_TICKS(100));
    err = mpu6050_register_read(dev_handle, MPU6050_PWR_MGMT_1_REG_ADDR, data, 1);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "No se pudo leer %s: %s", "PWR_MGMT", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "%s = 0x%02X", "PWM_MGMT", data[0]);

    err = mpu6050_register_write_byte(dev_handle, MPU6050_CONFIG, 0x01); // Activamos el filtro pasabajo (Chequear config)
    err = mpu6050_register_write_byte(dev_handle, SMPLRT_DIV, 0x04);
    err = mpu6050_register_write_byte(dev_handle, MPU6050_FIFO_RESET, 0x04);
    err = mpu6050_register_write_byte(dev_handle, MPU6050_FIFO_EN, 0x40);

    err = mpu6050_register_write_byte(dev_handle, MPU6050_FIFO_EN, 0x78);

    err = mpu6050_register_read(dev_handle, MPU6050_FIFO_EN, data, 1);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "No se pudo LEER %s: %s", "FIFO_EN", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "%s = 0x%02X", "FIFO_EN", data[0]);

    /*
    #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || \
        defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)

        ESP_ERROR_CHECK(
            uros_network_interface_initialize());

    #endif

    xTaskCreate(
    micro_ros_task,
    "micro_ros_task",
    MICRO_ROS_APP_STACK,
    NULL,
    MICRO_ROS_APP_TASK_PRIO,
    NULL);
    */
}
