#include <stdint.h>
#include <unistd.h>
#include <stdbool.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "sdkconfig.h"

#include "driver/mpu6050.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/pose_stamped.h>

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
#define TIMER_PERIOD_MS 100

#ifndef MICROROS_NAMESPACE
#define MICROROS_NAMESPACE ""
#endif

#ifndef DOMAIN_ID
#define DOMAIN_ID 0
#endif

static const char *TAG = "micro_ros";
static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;

static rcl_publisher_t pose_publisher;
static geometry_msgs__msg__PoseStamped pose_msg;

static TaskHandle_t s_mpu_task = NULL;

esp_err_t mpu_write(uint8_t reg, uint8_t data){
    return mpu6050_register_write_byte(dev_handle, reg, data);
}

esp_err_t mpu_read(uint8_t reg, uint8_t * data, uint8_t len){
    return mpu6050_register_read(dev_handle, reg, data, len);
}

static void IRAM_ATTR mpu_isr(void *arg)
{
    BaseType_t woken = pdFALSE;
    if (s_mpu_task) {
        vTaskNotifyGiveFromISR(s_mpu_task, &woken);
    }
    portYIELD_FROM_ISR(woken);
}

static esp_err_t mpu_init(void)
{
    i2c_master_init(&bus_handle, &dev_handle);

    /* Identify */
    uint8_t who = 0;
    ESP_RETURN_ON_ERROR(mpu_read(MPU6050_WHO_AM_I_REG_ADDR, &who, 1), TAG, "WHO_AM_I read failed");
    ESP_LOGI(TAG, "MPU WHO_AM_I = 0x%02X", who);
    if (who != 0x68) {
        ESP_LOGW(TAG, "Unexpected WHO_AM_I (expected 0x68). Clone chips may differ; continuing.");
    }

    /* Reset, then wake up with gyro X PLL clock */
    ESP_RETURN_ON_ERROR(mpu_write(MPU6050_PWR_MGMT_1_REG_ADDR, 0x80), TAG, "reset failed");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(mpu_write(MPU6050_PWR_MGMT_1_REG_ADDR, 0x01), TAG, "wake failed");
    vTaskDelay(pdMS_TO_TICKS(100));

    /* DLPF = 4 (21 Hz accel / 20 Hz gyro) -> gyro output rate = 1 kHz */
    ESP_RETURN_ON_ERROR(mpu_write(MPU6050_CONFIG, 0x04), TAG, "CONFIG failed");

    /* Sample rate = 1000 / (1 + SMPLRT_DIV) */
    uint8_t div = (uint8_t)(1000 / CONFIG_SAMPLE_RATE - 1);
    ESP_RETURN_ON_ERROR(mpu_write(SMPLRT_DIV, div), TAG, "SMPLRT_DIV failed");
    ESP_LOGI(TAG, "MPU sample rate: %d Hz (SMPLRT_DIV=%u)",
             1000 / (1 + div), div);

    ESP_RETURN_ON_ERROR(mpu_write(MPU6050_GYRO_CONFIG, 0x00), TAG, "GYRO_CONFIG failed");   /* +-250 dps */
    ESP_RETURN_ON_ERROR(mpu_write(MPU6050_ACCEL_CONFIG, 0x00), TAG, "ACCEL_CONFIG failed"); /* +-2 g */

    /* INT pin: active high, push-pull, 50 us pulse, cleared on any read */
    ESP_RETURN_ON_ERROR(mpu_write(MPU6050_INT_PIN_CFG, 0x10), TAG, "INT_PIN_CFG failed");

    /* FIFO: reset enabnle and configure */
    ESP_RETURN_ON_ERROR(mpu_write(MPU6050_FIFO_RESET, 0x04), TAG, "fifo reset failed");
    ESP_RETURN_ON_ERROR(mpu_write(MPU6050_FIFO_RESET, 0x40), TAG, "fifo enable failed");
    ESP_RETURN_ON_ERROR(mpu_write(MPU6050_FIFO_EN,    0x78), TAG, "fifo config failed"); // Enable Gyro and Accel

    /* ESP32 GPIO for INT (set up BEFORE enabling the interrupt on the MPU) */
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << CONFIG_INT_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(CONFIG_INT_PIN, mpu_isr, NULL));

    /* Enable Data Ready interrupt */
    ESP_RETURN_ON_ERROR(mpu_write(MPU6050_INT_ENABLE, 0x01), TAG, "INT_ENABLE failed");

    return ESP_OK;
}

static void mpu_task(void *arg)
{
    uint8_t fifo[FIFO_BURST_LEN];
    uint8_t fifo_count[2];
    uint32_t irq_count = 0;
    int64_t t_start = esp_timer_get_time();
    mpu6050_data_t * data_p = malloc(sizeof(mpu6050_data_t));

    while (1) {
        /* Wait for the data-ready interrupt (timeout lets us detect a dead INT line) */
        uint32_t n = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
        if (n > 0) {
            irq_count += n;
            if (mpu_read(MPU6050_FIFO_COUNT_H, fifo_count, 2) == RCL_RET_OK) {
                if ((fifo_count[0] << 8 | fifo_count[1]) > FIFO_BURST_LEN) {
                    ESP_LOGW(TAG, "FIFO count overflowing: %d", fifo_count[0] << 8 | fifo_count[1]);
                }
            }
            if (mpu_read(MPU6050_FIFO_R_W, fifo, FIFO_BURST_LEN) == RCL_RET_OK) {
                int16_t v[6];
                for (int i = 0; i < 6; i++) {
                    v[i] = (int16_t)((fifo[2 * i] << 8) | fifo[2 * i + 1]);
                }
            	data_p->ax = v[0] / ACCEL_LSB_PER_G;
            	data_p->ay = v[1] / ACCEL_LSB_PER_G;
            	data_p->az = v[2] / ACCEL_LSB_PER_G;
            	data_p->gx = v[3] / GYRO_LSB_PER_DPS;
            	data_p->gy = v[4] / GYRO_LSB_PER_DPS;
            	data_p->gz = v[5] / GYRO_LSB_PER_DPS;

                /* LuUUuUuUUuUUuUUu ACA PROCESAMOS DATOS Y GENERAMOS CUATERNION */

                /* ACA PUBLICAMOS LOS DATOS UNA VEZ PROCESADOS */
            } else {
                ESP_LOGW(TAG, "MPU read failed");
            }
        }

        int64_t now = esp_timer_get_time();
        if (now - t_start >= 1000000) {
            float rate = irq_count / ((now - t_start) / 1e6f);
            printf("MPU INT rate: %.1f Hz | Acc[g] %.3f %.3f %.3f | Gyro[dps] %.2f %.2f %.2f\n",
                   rate, data_p->ax, data_p->ay, data_p->az,
                   data_p->gx, data_p->gy, data_p->gz);
            if (irq_count == 0) {
                ESP_LOGW(TAG, "No MPU interrupts received - check INT wiring (GPIO%d)", CONFIG_INT_PIN);
            }
            irq_count = 0;
            t_start = now;
        }
    }
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

    RCCHECK(rcl_init_options_init(
        &init_options,
        allocator);
    )

    RCCHECK(rcl_init_options_set_domain_id(
        &init_options,
        DOMAIN_ID);
    )

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE

    rmw_init_options_t *rmw_options =
        rcl_init_options_get_rmw_init_options(
            &init_options);

    RCCHECK(rmw_uros_options_set_udp_address(
        CONFIG_MICRO_ROS_AGENT_IP,
        CONFIG_MICRO_ROS_AGENT_PORT,
        rmw_options);
    )

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

    RCCHECK(rclc_node_init_default(
        &node,
        "microros_node",
        MICROROS_NAMESPACE,
        &support);
    )

    RCCHECK(rclc_publisher_init_default(
        &pose_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(
            geometry_msgs,
            msg,
            PoseStamped
        ),
        "pose"
    );)

    /* TODO: Borrar si no usamos el timer
    rcl_timer_t timer =
        rcl_get_zero_initialized_timer();

     RCCHECK(rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS(TIMER_PERIOD_MS),
        timer_callback,
        true);
    ) */

    rclc_executor_t executor =
        rclc_executor_get_zero_initialized_executor();

    RCCHECK(rclc_executor_init(
        &executor,
        &support.context,
        1,
        &allocator);
    )

    /* TODO: Borrar si no usamos el timer
    RCCHECK(rclc_executor_add_timer(
        &executor,
        &timer
    );
    }
    */

    xTaskCreate(mpu_task, "mpu_task", 4096, NULL, 10, &s_mpu_task);
    if (mpu_init() != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 init failed - check wiring/address");
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
}
