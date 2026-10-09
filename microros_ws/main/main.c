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
#include "accelerometer.h"
#include "gyroscope.h"
#include "quaternion.h"
#include "complementary_filter.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/pose_stamped.h>
#include <math.h>

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

    uint8_t who = 0;
    ESP_RETURN_ON_ERROR(mpu6050_register_read(dev_handle, MPU6050_WHO_AM_I_REG_ADDR, &who, 1), TAG, "WHO_AM_I");
    ESP_LOGI(TAG, "MPU WHO_AM_I = 0x%02X", who);

    ESP_RETURN_ON_ERROR(mpu6050_dmp_prepare(dev_handle), TAG, "DMP init");

    /* Configurar GPIO e ISR antes de habilitar las interrupciones del MPU. */
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << CONFIG_INT_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(CONFIG_INT_PIN, mpu_isr, NULL));

    return mpu6050_dmp_start(dev_handle);
}

static void mpu_task(void *arg)
{
    uint8_t pkt[DMP_PACKET_SIZE];
    uint16_t count = 0;
    uint32_t irq_count = 0, pkt_count = 0;
    int64_t t_start = esp_timer_get_time();

    float qw = 1, qx = 0, qy = 0, qz = 0;
    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;

    accelerometer_output_t accel;
    gyroscope_output_t gyro;
    quaternion_t orientation;

    quaternion_identity(&orientation);
    const float dt_s = 0.005f;

    while (1) {
        /* Espera la interrupcion del DMP (el timeout permite detectar INT muerta) */
        uint32_t n = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
        if (n > 0) {
            irq_count += n;

            if (mpu6050_dmp_read_fifo_count(dev_handle, &count) == ESP_OK) {
                if (count >= 1024) {                     /* FIFO desbordado: paquetes desalineados */
                    ESP_LOGW(TAG, "FIFO overflow, reset");
                    mpu6050_dmp_reset_fifo(dev_handle);
                    count = 0;
                }

                /* Pueden haber varios paquetes acumulados: leer todos los completos */
                while (count >= DMP_PACKET_SIZE) {
                    if (mpu6050_dmp_read_fifo_packet(dev_handle, pkt) != ESP_OK) {
                        ESP_LOGW(TAG, "FIFO read failed");
                        break;
                    }
                    count -= DMP_PACKET_SIZE;
                    pkt_count++;

                    /* Cuaternion del DMP: 4 x int32 big-endian en Q30 (bytes 0..15) */
                    int32_t q[4];
                    for (int i = 0; i < 4; i++) {
                        q[i] = (int32_t)(((uint32_t)pkt[4*i]   << 24) | ((uint32_t)pkt[4*i+1] << 16) |
                                         ((uint32_t)pkt[4*i+2] <<  8) |  (uint32_t)pkt[4*i+3]);
                    }
                    qw = q[0] / Q30;  qx = q[1] / Q30;
                    qy = q[2] / Q30;  qz = q[3] / Q30;

                    /* Crudo de la MISMA muestra: gyro en 16/20/24, accel en 28/32/36 */
                    gx = (int16_t)((pkt[16] << 8) | pkt[17]) / GYRO_LSB_PER_DPS_DMP;
                    gy = (int16_t)((pkt[20] << 8) | pkt[21]) / GYRO_LSB_PER_DPS_DMP;
                    gz = (int16_t)((pkt[24] << 8) | pkt[25]) / GYRO_LSB_PER_DPS_DMP;
                    ax = (int16_t)((pkt[28] << 8) | pkt[29]) / ACCEL_LSB_PER_G_DMP;
                    ay = (int16_t)((pkt[32] << 8) | pkt[33]) / ACCEL_LSB_PER_G_DMP;
                    az = (int16_t)((pkt[36] << 8) | pkt[37]) / ACCEL_LSB_PER_G_DMP;

                    accelerometer_process(
                        ax,
                        ay,
                        az,
                        &accel
                    );

                    gyroscope_process(
                        gx,
                        gy,
                        gz,
                        dt_s,
                        &gyro
                    );

                    complementary_filter_update(
                        &orientation,
                        &accel,
                        &gyro,
                        dt_s,
                        COMPLEMENTARY_FILTER_DEFAULT_ALPHA
                    );

                    
                }
            }
        }

        int64_t now = esp_timer_get_time();
        if (now - t_start >= 1000000) {
            float dt = (now - t_start) / 1e6f;
            printf(
                "INT %.1f Hz | pkt %.1f Hz | "
                "DMP[wxyz] %.3f %.3f %.3f %.3f | "
                "CALC[wxyz] %.3f %.3f %.3f %.3f | "
                "Acc[g] %.3f %.3f %.3f | "
                "Gyro[dps] %.2f %.2f %.2f\n",

                irq_count / dt,
                pkt_count / dt,

                qw, qx, qy, qz,

                orientation.w,
                orientation.x,
                orientation.y,
                orientation.z,

                ax, ay, az,
                gx, gy, gz
            );
            
            if (irq_count == 0) {
                ESP_LOGW(TAG, "No MPU interrupts received - check INT wiring (GPIO%d)", CONFIG_INT_PIN);
            }
            irq_count = 0;
            pkt_count = 0;
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

    xTaskCreate(mpu_task, "mpu_task", 4096, NULL, 10, &s_mpu_task);
    if (mpu_init() != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 init failed - check wiring/address");
    }

    /*
    #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || \
        defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)

        ESP_ERROR_CHECK(
            uros_network_interface_initialize());

    #endif
    */

    /*
    xTaskCreate(
    micro_ros_task,
    "micro_ros_task",
    MICRO_ROS_APP_STACK,
    NULL,
    MICRO_ROS_APP_TASK_PRIO,
    NULL);
    */
}