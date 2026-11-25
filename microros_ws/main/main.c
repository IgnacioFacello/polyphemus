#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

// Drivers (drivers/) y logica del robot (logica/)
#include "red.h"
#include "vl53l0x_drv.h"
#include "hc_sr04.h"

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
#define MEDICION_TASK_STACK  4096
#define MEDICION_TASK_PRIO   4
#define MEDICION_PERIODO_MS  100   // 10 Hz. El ciclo real tarda ~50 ms (láser y ultrasonido en paralelo)

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
// TAREA DE MEDICIÓN: ULTRASONIDO (HC-SR04) vs LÁSER (VL53L0X)
// =========================================================
// Va en su propia tarea porque las mediciones bloquean (~50 ms) y frenarían el executor.
// Los dos miden en la misma ventana de tiempo: se dispara el ultrasonido, mientras el sonido
// va y vuelve (el RMT captura el eco por hardware) se mide con el láser, y después se lee el eco.
static hcsr04_handle_t ultrasonido = NULL;

static const char *us_estado_str(esp_err_t e)
{
    switch (e) {
    case ESP_OK:                   return "OK";
    case ESP_ERR_TIMEOUT:          return "SIN_ECO";      // no llegó eco: cableado
    case ESP_ERR_INVALID_RESPONSE: return "FUERA_RANGO";  // < 2 cm o > 400 cm
    default:                       return "ERROR";
    }
}

static void medicion_task(void * arg)
{
    // Una línea por medición, con prefijo "CSV," para filtrarla del resto del log:
    //   idf.py monitor | grep "^CSV," > mediciones.csv
    // us_eco_us = tiempo de ida y vuelta del sonido; laser_dur_us = lo que tardó laser_medir()
    printf("CSV,t_ms,us_mm,us_estado,us_eco_us,laser_mm,laser_estado,laser_dur_us\n");

    TickType_t ultimo_ciclo = xTaskGetTickCount();
    while (1) {
        int64_t t0 = esp_timer_get_time();

        // 1) Disparar el ultrasonido (vuelve enseguida, no espera el eco)
        esp_err_t us_err = hcsr04_start(ultrasonido);

        // 2) Mientras vuela el sonido, medir con el láser (~33 ms)
        uint16_t laser_mm = 0;
        int64_t t_laser = esp_timer_get_time();
        laser_estado_t laser_est = laser_medir(&laser_mm);
        int64_t laser_dur_us = esp_timer_get_time() - t_laser;

        // 3) Leer el eco del ultrasonido
        float us_cm = 0.0f;
        uint32_t us_eco_us = 0;
        if (us_err == ESP_OK) {
            us_err = hcsr04_wait_cm(ultrasonido, &us_cm, &us_eco_us);
        }
        float us_mm = us_cm * 10.0f;   // todo en mm

        printf("CSV,%lld,%.1f,%s,%lu,%u,%s,%lld\n",
               (long long)(t0 / 1000), us_mm, us_estado_str(us_err), (unsigned long)us_eco_us,
               laser_mm, laser_estado_str(laser_est), (long long)laser_dur_us);

        // Periodo fijo: descuenta lo que ya tardó el ciclo (vTaskDelay esperaría 100 ms extra)
        vTaskDelayUntil(&ultimo_ciclo, pdMS_TO_TICKS(MEDICION_PERIODO_MS));
    }
}

void app_main(void)
{
    // INICIALIZACIONES START
    // Prender los sensores (una sola vez). Si alguno falla, sigue andando micro-ROS sin medir.
    bool laser_ok = (laser_init() == ESP_OK);
    if (!laser_ok) {
        ESP_LOGE(TAG, "No se pudo iniciar el láser");
    }
    hcsr04_config_t us_cfg = HCSR04_CONFIG_DEFAULT();   // pines: menuconfig -> "Ultrasonido (HC-SR04)"
    esp_err_t us_err = hcsr04_new(&us_cfg, &ultrasonido);
    if (us_err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo iniciar el ultrasonido: %s", esp_err_to_name(us_err));
    }
    // Opcional: más tiempo por medición = menos ruido (por defecto ~33 ms).
    // 66 ms deja al láser a la misma tasa que el HC-SR04 (>60 ms entre mediciones).
    // laser_set_budget_us(66000);
    // INICIALIZACIONES END
    ESP_ERROR_CHECK(red_wifi_init());
    xTaskCreate(micro_ros_task, "micro_ros_task", MICRO_ROS_TASK_STACK, NULL, MICRO_ROS_TASK_PRIO, NULL);
    if (laser_ok && us_err == ESP_OK) {
        xTaskCreate(medicion_task, "medicion_task", MEDICION_TASK_STACK, NULL, MEDICION_TASK_PRIO, NULL);
    }
}
