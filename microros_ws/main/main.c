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

#include <custom_interfaces/srv/vector_rotate.h>

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


static const char *TAG = "micro_ros";

float pi = acos(-1.0);

void multiplicar_matriz_vector(float M[3][3], float V[3], float salida[3]) {
    for (int i = 0; i < 3; i++) {
        salida[i] = 0.0;
        for (int j = 0; j < 3; j++) {
            salida[i] += M[i][j] * V[j];
        }
    }
}

void service_callback(const void * req, void * res){
  custom_interfaces__srv__VectorRotate_Request * req_in = (custom_interfaces__srv__VectorRotate_Request *) req;
  custom_interfaces__srv__VectorRotate_Response * res_in = (custom_interfaces__srv__VectorRotate_Response *) res;


  float x = (float) req_in->x;
  float y = (float) req_in->y;
  float z = (float) req_in->z;
  // Vector que queremos rotar
  float V[3] = {x, y, z};

  float alpha_deg = (float) req_in->alpha;
  float beta_deg = (float) req_in->beta;
  float phi_deg = (float) req_in->phi;

  uint cantidad = (uint) req_in->steps;

  printf("Service request value: (%.2f, %.2f, %.2f) (%2.f, %2.f) %2.f %d.\n",
            x, y, z,
            alpha_deg, beta_deg, phi_deg,
            cantidad
  );

  float paso_g = phi_deg / cantidad; // Paso de microrotación en grados

  float deg_to_rad = pi/180.0;
  float alfa = alpha_deg * deg_to_rad;
  float beta = beta_deg * deg_to_rad;
  float fi = phi_deg * deg_to_rad;
  float paso = paso_g * deg_to_rad;

  // Componentes del eje de rotación
  float nx = sin(beta) * cos(alfa);
  float ny = sin(beta) * sin(alfa);
  float nz = cos(beta);
  float cosfi = cos(fi);
  float sinfi = sin(fi);

  // Matriz para rotación total
  float M1[3][3] = {
      {
          nx * nx + (1.0 - nx * nx) * cosfi,
          nx * ny * (1.0 - cosfi) + nz * sinfi,
          nx * nz * (1.0 - cosfi) - ny * sinfi
      },
      {
          nx * ny * (1.0 - cosfi) - nz * sinfi,
          ny * ny + (1.0 - ny * ny) * cosfi,
          ny * nz * (1.0 - cosfi) + nx * sinfi
      },
      {
          nx * nz * (1.0 - cosfi) + ny * sinfi,
          ny * nz * (1.0 - cosfi) - nx * sinfi,
          nz * nz + (1.0 - nz * nz) * cosfi
      }
  };

  // Matriz de rotación simplificada MICRO ROTACION
  float M_micro[3][3] = {
      {1.0,       nz * paso, -ny * paso},
      {-nz * paso, 1.0,       nx * paso},
      {ny * paso, -nx * paso, 1.0}
  };

  float V_micro[3] = {V[0], V[1], V[2]};
  float temp[3];

  for (int i = 0; i < cantidad; i++) {
      multiplicar_matriz_vector(M_micro, V_micro, temp);

      V_micro[0] = temp[0];
      V_micro[1] = temp[1];
      V_micro[2] = temp[2];
  }

  // Rotamos el vector de forma exacta
  float V_rotado_exacto[3];
  multiplicar_matriz_vector(M1, V, V_rotado_exacto);

  //
   res_in->fullx = V_rotado_exacto[0];
   res_in->fully = V_rotado_exacto[1];
   res_in->fullz = V_rotado_exacto[2];

   res_in->micrx = V_micro[0];
   res_in->micry = V_micro[1];
   res_in->micrz = V_micro[2];
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

    // create service
    rcl_service_t service;
    RCCHECK(rclc_service_init_default(&service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(custom_interfaces, srv, VectorRotate), "/vector_rotate"));

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

    custom_interfaces__srv__VectorRotate_Response res;
    custom_interfaces__srv__VectorRotate_Request req;
    RCCHECK(rclc_executor_add_service(&executor, &service, &req, &res, service_callback));
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

    // Free resources
    RCCHECK(rcl_service_fini(&service, &node));
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
