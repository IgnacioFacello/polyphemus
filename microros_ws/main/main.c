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

SemaphoreHandle_t xSemaphore = NULL;
rcl_ret_t mpu6050_receive_fifo (
    i2c_master_dev_handle_t dev_handle,
	mpu6050_data_t *data_p
) {
	uint8_t fifo[FIFO_BURST_LEN];
	rcl_ret_t err = RCL_RET_OK;

	// Read specified FIFO buffer size (depends on configuration set)
	for (uint8_t i = 0; i < FIFO_BURST_LEN; ++i) {
		if ((err = mpu6050_register_read(dev_handle, MPU6050_FIFO_R_W,
		fifo+i, 1)) != RCL_RET_OK) {
			break;
		}
	}

	// Configure data structure
	data_p->ax = (int16_t)fifo[0]  << 8  | (int16_t)fifo[1];
	data_p->ay = (int16_t)fifo[2]  << 8  | (int16_t)fifo[3];
	data_p->az = (int16_t)fifo[4]  << 8  | (int16_t)fifo[5];
	data_p->gx = (int16_t)fifo[6]  << 8  | (int16_t)fifo[7];
	data_p->gy = (int16_t)fifo[8]  << 8  | (int16_t)fifo[9];
	data_p->gz = (int16_t)fifo[10] << 8  | (int16_t)fifo[11];

	return err;
}

void leer_fifo_count(uint16_t *count)
{
    uint8_t buffer[2];

    RCCHECK(mpu6050_register_read(dev_handle, MPU6050_FIFO_COUNT_H, buffer, 2);)

    // buffer[0] tiene FIFO_COUNT_H
    // buffer[1] tiene FIFO_COUNT_L
    *count = ((uint16_t)buffer[0] << 8) | buffer[1];
}

void IRAM_ATTR button_isr_handler(void* arg) {
  xSemaphoreGiveFromISR(xSemaphore, NULL);
}

rcl_ret_t configure_mpu(i2c_master_dev_handle_t dev_handle){
    if (dev_handle == NULL)
    {
        ESP_LOGE(TAG, "Error initializing I2C");
        return RCL_RET_ERROR;
    }

    RCCHECK(mpu6050_register_write_byte(dev_handle, MPU6050_PWR_MGMT_1_REG_ADDR, 0x01)); // sacar de sleep + elegir reloj
    vTaskDelay(pdMS_TO_TICKS(100));
    RCCHECK(mpu6050_register_write_byte(dev_handle, MPU6050_CONFIG, 0x01)); // Activamos el filtro pasabajo (Chequear config)
    RCCHECK(mpu6050_register_write_byte(dev_handle, SMPLRT_DIV, 0x04));
    RCCHECK(mpu6050_register_write_byte(dev_handle, MPU6050_FIFO_RESET, 0x04));
    RCCHECK(mpu6050_register_write_byte(dev_handle, MPU6050_FIFO_RESET, 0x40));

    RCCHECK(mpu6050_register_write_byte(dev_handle, MPU6050_FIFO_EN, 0x78));

    return RCL_RET_OK;
}

void sensor_task(void* arg) {
    mpu6050_data_t * data_p = malloc(sizeof(mpu6050_data_t));

    configure_mpu(dev_handle);

    for(;;) {
    if(xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
        /* NOTE: Data is writen to the FIFO in order of register number (lowest to highest)
         * 0-5 ACCELEROMETER 6-11 GYROSCOPE
         */
        mpu6050_receive_fifo(dev_handle, data_p);
        ESP_LOGI(TAG, "Accelerometer: ax=%d, ay=%d, az=%d", data_p->ax, data_p->ay, data_p->az);
        ESP_LOGI(TAG, "Gyroscope: gx=%d, gy=%d, gz=%d", data_p->gx, data_p->gy, data_p->gz);
        //RCSOFTCHECK(rcl_publish(&pose_publisher, &pose_msg, NULL);)
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
    xSemaphore = xSemaphoreCreateBinary();

	gpio_set_direction(CONFIG_INT_PIN, GPIO_MODE_INPUT);

    gpio_set_intr_type(
        CONFIG_INT_PIN, // gpio_num_t,
        GPIO_INTR_POSEDGE // gpio_int_type_t
    );

    i2c_master_init(&bus_handle, &dev_handle);
    if (dev_handle == NULL)
    {
        ESP_LOGE(TAG, "Error initializing I2C");
        return;
    }
    ESP_LOGI(TAG, "I2C initialized successfully");

    /*
    #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || \
        defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)

        ESP_ERROR_CHECK(
            uros_network_interface_initialize());

    #endif
    */

    xTaskCreate(sensor_task, "sensor_task",
        2048, NULL,
        10, NULL
    );

    gpio_install_isr_service(0);

    gpio_isr_handler_add(
        CONFIG_INT_PIN,
        button_isr_handler,
        NULL
    );

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
