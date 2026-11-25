#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Drivers (drivers/) y logica del robot (logica/)
#include "red.h"
#include "vl53l0x_drv.h"

// Librerías base de micro-ROS
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/empty.h>

// Stack y prioridad de las tareas
#define MICRO_ROS_TASK_STACK 16000
#define MICRO_ROS_TASK_PRIO  5
#define LASER_TASK_STACK     4096
#define LASER_TASK_PRIO      4
#define LASER_PERIODO_MS     100   // ~33 ms de medición + 100 ms de espera ≈ 7,5 veces por segundo

#define ENTIDAD_TIMEOUT_MS 5000   // espera máxima del agente al crear cada entidad
#define PING_PERIODO_MS 1000  // cada cuánto se chequea que el agente siga vivo

static const char *TAG = "polyphemus"; // NOMBRE DEL PROYECTO
#define ROSDOMAIN_ID 0                 // ID del dominio

static const char *volatile estado_actual = "ESPERANDO ORDENES";

// IP donde se busca/usa el agente. Con PROMETEO_AGENTE_AUTO se reemplaza por la que responda.
static char micro_ros_agent_ip_buf[16] = CONFIG_MICRO_ROS_AGENT_IP;
char * micro_ros_agent_ip = micro_ros_agent_ip_buf;

// =========================================================
// OBJETOS DE MICRO-ROS
// =========================================================
static rcl_allocator_t allocator;
static rcl_init_options_t init_options;
static rmw_init_options_t *rmw_options;
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;
// ==== pubs, subs y msgs START ====
/* ejemplo:
 * static rcl_publisher_t pub;
 * static rcl_subscription_t sub;
*/
// ==== pubs, subs y msgs END   ====

// ==== timers START ====
// ejemplo: static rcl_timer_t timer;
// ==== timers END   ====

#define RCRET(fn) { rcl_ret_t temp_rc = fn; if (temp_rc != RCL_RET_OK) { \
    ESP_LOGE(TAG, "Fallo en linea %d: rc=%d", __LINE__, (int)temp_rc); return false; } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { \
    ESP_LOGW(TAG, "Fallo en linea %d: %d. Continuando.", __LINE__, (int)temp_rc); } }

// =========================================================
// CREAR / DESTRUIR micro-ROS
// =========================================================
static bool crear_entidades(void)
{
    RCRET(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    // Por WiFi cada entidad espera la respuesta del agente: el default (1000 ms) no alcanza
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_creation_session_timeout(rmw_context, ENTIDAD_TIMEOUT_MS);

    // ==== Inicializacion del nodo  ====
    RCRET(rclc_node_init_default(&node, "body", TAG, &support));

    // ==== Inicializacion de pubs START  ====
    /* ejemplo:
     * RCRET(rclc_publisher_init_<qos>(&pub, &node,
     *     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, <Tipo>), "<nombre>"));
     */

    // ==== Inicializacion de pubs END    ====

    // ==== Inicializacion de subs START  ====
    /* ejemplo:
     * RCRET(rclc_subscription_init_<qos>(&sub, &node,
           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, <Tipo>), "<nombre>"));
     */

    // ==== Inicializacion de subs END    ====

    // ==== Inicializacion de timers START  ====
    /* ejemplo:
     * RCRET(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(500), timer_callback));
     */

    // ==== Inicializacion de timers END    ====

    // ==== Inicializacion de executor  ====
    executor = rclc_executor_get_zero_initialized_executor();
    RCRET(rclc_executor_init(&executor, &support.context, 7, &allocator));  // 2 timers + 5 suscripciones

    // ==== Agregar al executor START  ====
    /* ejemplo:
     * RCRET(rclc_executor_add_timer(&executor, &timer));
     * RCRET(rclc_executor_add_subscription(&executor, &sub, &msg,
     *      &callback, ON_NEW_DATA));
     */
    // ==== Agregar al executor END    ====

    return true;
}

static void destruir_entidades(void)
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    // ==== Limpieza de subs START  ====
    // ejemplo: RCSOFTCHECK(rcl_subscription_fini(&sub, &node));

    // ==== Limpieza de subs END    ====

    // ==== Limpieza de pubs START  ====
    //ejemplo: RCSOFTCHECK(rcl_publisher_fini(&pub, &node));

    // ==== Limpieza de pubs END    ====

    // ==== Limpieza de timers START  ====
    // ejemplo: RCSOFTCHECK(rcl_timer_fini(&timer));

    // ==== Limpieza de timers END    ====

    // ==== Limpieza de otros START  ====
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));
    // ==== Limpieza de otros END    ====
}

// =========================================================
// TAREA DE MICRO-ROS con reconexión
// =========================================================
typedef enum {
    ESPERANDO_AGENTE,
    AGENTE_DISPONIBLE,
    CONECTADO,
    DESCONECTADO
} estado_uros_t;

static void micro_ros_task(void * arg)
{
    allocator = rcl_get_default_allocator();
    init_options = rcl_get_zero_initialized_init_options();
    RCSOFTCHECK(rcl_init_options_init(&init_options, allocator));
    RCSOFTCHECK(rcl_init_options_set_domain_id(&init_options, ROSDOMAIN_ID));   // mismo ROS_DOMAIN_ID que el agente y la PC
    rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    rmw_uros_options_set_udp_address(micro_ros_agent_ip, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options);

    // ==== Configuracion de los mensajes START  ====
    // ==== Configuracion de los mensajes END    ====

    estado_uros_t estado = ESPERANDO_AGENTE;
    int ms_desde_ping = 0;

    while (1) {
        switch (estado) {

        case ESPERANDO_AGENTE:
            if (rmw_uros_ping_agent_options(200, 1, rmw_options) == RMW_RET_OK) {
                ESP_LOGI(TAG, "Agente encontrado");
                estado = AGENTE_DISPONIBLE;
            } else {
                ESP_LOGW(TAG, "Buscando agente por barrida...");
                if (red_buscar_agente(rmw_options, micro_ros_agent_ip_buf, sizeof(micro_ros_agent_ip_buf))) {
                    ESP_LOGI(TAG, "Agente encontrado por barrida en %s", micro_ros_agent_ip);
                    estado = AGENTE_DISPONIBLE;
                    break;
                }
                // la barrida dejó cambiada la dirección: volver a la última conocida
                rmw_uros_options_set_udp_address(micro_ros_agent_ip, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options);
                vTaskDelay(pdMS_TO_TICKS(500));
            }
            break;

        case AGENTE_DISPONIBLE:
            if (crear_entidades()) {
                ESP_LOGI(TAG, "CONECTADO");
                estado = CONECTADO;
                ms_desde_ping = 0;
            } else {
                destruir_entidades();
                estado = ESPERANDO_AGENTE;
            }
            break;

        case CONECTADO:
            if (ms_desde_ping >= PING_PERIODO_MS) {
                ms_desde_ping = 0;
                if (rmw_uros_ping_agent(100, 3) != RMW_RET_OK) {
                    estado = DESCONECTADO;
                    break;
                }
            }
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
            // ==== Tareas Periodicas START  ====
            // ==== Tareas Periodicas END    ====
            vTaskDelay(pdMS_TO_TICKS(10));
            ms_desde_ping += 20;
            break;

        case DESCONECTADO:
            ESP_LOGE(TAG, "Agente PERDIDO -> FRENO y reconexion");
            // ==== Tareas de Limpieza START  ====
            // ==== Tareas de Limpieza END    ====
            destruir_entidades();
            estado_actual = "ESPERANDO ORDENES";
            estado = ESPERANDO_AGENTE;
            break;
        }
    }
}

// =========================================================
// TAREA DEL LÁSER (VL53L0X)
// =========================================================
// Va en su propia tarea porque laser_medir() bloquea ~33 ms y frenaría el executor.
static void laser_task(void * arg)
{
    while (1) {
        uint16_t mm;
        laser_estado_t e = laser_medir(&mm);
        if (e == LASER_OK) {
            ESP_LOGI(TAG, "Distancia: %u mm  (señal %.2f MCPS)", mm, laser_ultima_senal_mcps());
        } else {
            ESP_LOGW(TAG, "Medición %s (leído: %u mm)", laser_estado_str(e), mm);
        }
        vTaskDelay(pdMS_TO_TICKS(LASER_PERIODO_MS));
    }
}

void app_main(void)
{
    // INICIALIZACIONES START
    // Prender el láser (una sola vez). Si falla, sigue andando micro-ROS sin láser.
    bool laser_ok = (laser_init() == ESP_OK);
    if (!laser_ok) {
        ESP_LOGE(TAG, "No se pudo iniciar el láser");
    }
    // Opcional: más tiempo por medición = menos ruido (por defecto ~33 ms).
    // 66 ms deja al láser a la misma tasa que el HC-SR04 (>60 ms entre mediciones).
    // laser_set_budget_us(66000);
    // INICIALIZACIONES END
    ESP_ERROR_CHECK(red_wifi_init());
    xTaskCreate(micro_ros_task, "micro_ros_task", MICRO_ROS_TASK_STACK, NULL, MICRO_ROS_TASK_PRIO, NULL);
    if (laser_ok) {
        xTaskCreate(laser_task, "laser_task", LASER_TASK_STACK, NULL, LASER_TASK_PRIO, NULL);
    }
}
