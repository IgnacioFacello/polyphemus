#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
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
#include <sensor_msgs/msg/range.h>

// Stack y prioridad de las tareas
#define MICRO_ROS_TASK_STACK 16000
#define MICRO_ROS_TASK_PRIO  5
#define MEDICION_TASK_STACK  4096
#define MEDICION_TASK_PRIO   4
// ==== Configuración de la medición (cambiar acá y reflashear) ====
// Regla: MEDICION_PERIODO_MS tiene que ser >= LASER_BUDGET_MS + ~15 ms (si no, el ciclo no entra)
#define LASER_BUDGET_MS      66    // tiempo por medición del láser: más = menos ruido, más lento (20 a 400)
#define MEDICION_PERIODO_MS  100   // cada cuánto se mide: 100 ms = 10 Hz
#define LASER_MAX_MM         1200  // max_range del láser en ROS (perfil por defecto); más lejos -> +Inf

#define ENTIDAD_TIMEOUT_MS 5000   // espera máxima del agente al crear cada entidad
#define PING_PERIODO_MS 1000  // cada cuánto se chequea que el agente siga vivo
#define SYNC_PERIODO_MS 30000 // cada cuánto se resincroniza la hora con el agente
#define SYNC_TIMEOUT_MS 1000  // espera máxima de la sincronización de hora

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
// Un tópico sensor_msgs/Range por sensor: /polyphemus/laser/range y /polyphemus/ultrasonido/range
static rcl_publisher_t pub_laser;
static rcl_publisher_t pub_us;
static sensor_msgs__msg__Range msg_laser;
static sensor_msgs__msg__Range msg_us;
static char frame_laser[] = "laser_link";        // memoria estática para el frame_id (sin malloc)
static char frame_us[]    = "ultrasonido_link";
// ==== pubs, subs y msgs END   ====

// Última medición de los dos sensores: medicion_task la deja, micro_ros_task la publica.
// Cola de largo 1 con xQueueOverwrite: siempre queda solo la más nueva.
typedef struct {
    int64_t t_us;     // momento de la medición (esp_timer, us desde el arranque)
    float   us_m;     // ultrasonido en metros (+Inf = sin objeto / fuera de rango, NaN = error)
    float   laser_m;  // láser en metros (misma convención, REP 117)
} medicion_t;
static QueueHandle_t cola_medicion = NULL;

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
    // default = reliable: compatible con PlotJuggler y ros2 topic echo
    RCRET(rclc_publisher_init_default(&pub_laser, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), "laser/range"));
    RCRET(rclc_publisher_init_default(&pub_us, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), "ultrasonido/range"));

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
    RCSOFTCHECK(rcl_publisher_fini(&pub_laser, &node));
    RCSOFTCHECK(rcl_publisher_fini(&pub_us, &node));

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

// Completa los campos fijos de un mensaje Range (se llama una sola vez)
static void configurar_range(sensor_msgs__msg__Range *msg, char *frame, uint8_t tipo,
                             float fov_rad, float min_m, float max_m)
{
    memset(msg, 0, sizeof(*msg));
    msg->header.frame_id.data     = frame;
    msg->header.frame_id.size     = strlen(frame);
    msg->header.frame_id.capacity = strlen(frame) + 1;
    msg->radiation_type = tipo;
    msg->field_of_view  = fov_rad;
    msg->min_range      = min_m;
    msg->max_range      = max_m;
}

// Publica la última medición (si hay una nueva) en los dos tópicos
static void publicar_medicion(void)
{
    medicion_t m;
    if (xQueueReceive(cola_medicion, &m, 0) != pdTRUE) return;   // no hay medición nueva

    // Stamp = hora de la PC en el momento de la medición (no de la publicación).
    // Sin sincronizar queda en 0 y PlotJuggler usa la hora de recepción.
    int32_t  sec = 0;
    uint32_t nsec = 0;
    if (rmw_uros_epoch_synchronized()) {
        int64_t ns = rmw_uros_epoch_nanos() - (esp_timer_get_time() - m.t_us) * 1000;
        sec  = (int32_t)(ns / 1000000000LL);
        nsec = (uint32_t)(ns % 1000000000LL);
    }

    msg_laser.header.stamp.sec     = sec;
    msg_laser.header.stamp.nanosec = nsec;
    msg_laser.range                = m.laser_m;
    RCSOFTCHECK(rcl_publish(&pub_laser, &msg_laser, NULL));

    msg_us.header.stamp.sec     = sec;
    msg_us.header.stamp.nanosec = nsec;
    msg_us.range                = m.us_m;
    RCSOFTCHECK(rcl_publish(&pub_us, &msg_us, NULL));
}

static void micro_ros_task(void * arg)
{
    allocator = rcl_get_default_allocator();
    init_options = rcl_get_zero_initialized_init_options();
    RCSOFTCHECK(rcl_init_options_init(&init_options, allocator));
    RCSOFTCHECK(rcl_init_options_set_domain_id(&init_options, ROSDOMAIN_ID));   // mismo ROS_DOMAIN_ID que el agente y la PC
    rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    rmw_uros_options_set_udp_address(micro_ros_agent_ip, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options);

    // ==== Configuracion de los mensajes START  ====
    // Láser VL53L0X: haz de ~25° (0,44 rad), 3 cm a 1,2 m con el perfil por defecto
    configurar_range(&msg_laser, frame_laser, sensor_msgs__msg__Range__INFRARED,  0.44f, 0.03f, LASER_MAX_MM / 1000.0f);
    // Ultrasonido HC-SR04: cono de ~15° (0,26 rad), 2 cm a 4 m
    configurar_range(&msg_us,    frame_us,    sensor_msgs__msg__Range__ULTRASOUND, 0.26f, 0.02f, 4.0f);
    // ==== Configuracion de los mensajes END    ====

    estado_uros_t estado = ESPERANDO_AGENTE;
    int ms_desde_ping = 0;
    int ms_desde_sync = 0;

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
                // Sincronizar la hora con el agente, para que los stamps sean la hora de la PC
                if (rmw_uros_sync_session(SYNC_TIMEOUT_MS) != RMW_RET_OK) {
                    ESP_LOGW(TAG, "No se pudo sincronizar la hora (stamps en 0)");
                }
                ms_desde_sync = 0;
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
            publicar_medicion();
            if (ms_desde_sync >= SYNC_PERIODO_MS) {   // el reloj del ESP32 deriva: resincronizar
                ms_desde_sync = 0;
                (void) rmw_uros_sync_session(SYNC_TIMEOUT_MS);
            }
            // ==== Tareas Periodicas END    ====
            vTaskDelay(pdMS_TO_TICKS(10));
            ms_desde_ping += 20;
            ms_desde_sync += 20;
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
// Qué sensores iniciaron bien: la tarea mide solo esos (el que falló sale como ERROR / NaN)
static bool laser_listo = false;
static bool us_listo = false;

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
        esp_err_t us_err = us_listo ? hcsr04_start(ultrasonido) : ESP_FAIL;

        // 2) Mientras vuela el sonido, medir con el láser (~LASER_BUDGET_MS)
        uint16_t laser_mm = 0;
        laser_estado_t laser_est = LASER_ERROR_I2C;   // si no está listo -> NaN en ROS
        int64_t t_laser = esp_timer_get_time();
        if (laser_listo) {
            laser_est = laser_medir(&laser_mm);
        }
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

        // Para ROS: pasar a metros con la convención de sensor_msgs/Range (REP 117):
        // +Inf = no detectó objeto / fuera de rango, NaN = error de medición
        medicion_t m = { .t_us = t0 };
        // Lecturas OK por encima del max_range declarado (LASER_MAX_MM) también son +Inf,
        // para no publicar valores fuera de [min_range, max_range]. El CSV conserva el valor crudo.
        if (laser_est == LASER_OK && laser_mm <= LASER_MAX_MM)              m.laser_m = laser_mm / 1000.0f;
        else if (laser_est == LASER_OK || laser_est == LASER_SIN_OBJETO)   m.laser_m = INFINITY;
        else                                                               m.laser_m = NAN;
        if (us_err == ESP_OK)                         m.us_m = us_cm / 100.0f;
        else if (us_err == ESP_ERR_INVALID_RESPONSE)  m.us_m = INFINITY;
        else                                          m.us_m = NAN;
        xQueueOverwrite(cola_medicion, &m);   // micro_ros_task la publica cuando puede

        // Periodo fijo: descuenta lo que ya tardó el ciclo (vTaskDelay esperaría 100 ms extra)
        vTaskDelayUntil(&ultimo_ciclo, pdMS_TO_TICKS(MEDICION_PERIODO_MS));
    }
}

void app_main(void)
{
    // INICIALIZACIONES START
    // Prender los sensores (una sola vez). Si alguno falla, sigue andando micro-ROS sin medir.
    laser_listo = (laser_init() == ESP_OK);
    if (!laser_listo) {
        ESP_LOGE(TAG, "No se pudo iniciar el láser");
    }
    hcsr04_config_t us_cfg = HCSR04_CONFIG_DEFAULT();   // pines: menuconfig -> "Ultrasonido (HC-SR04)"
    esp_err_t us_err = hcsr04_new(&us_cfg, &ultrasonido);
    us_listo = (us_err == ESP_OK);
    if (!us_listo) {
        ESP_LOGE(TAG, "No se pudo iniciar el ultrasonido: %s", esp_err_to_name(us_err));
    }
    // Opcional: más tiempo por medición = menos ruido (por defecto ~33 ms).
    // 66 ms deja al láser a la misma tasa que el HC-SR04 (>60 ms entre mediciones).
    if (laser_listo) {
        esp_err_t e = laser_set_budget_us(LASER_BUDGET_MS * 1000);   // la función recibe microsegundos
        if (e != ESP_OK) {
            ESP_LOGW(TAG, "No se pudo poner el budget en %d ms: %s", LASER_BUDGET_MS, esp_err_to_name(e));
        }
    }
    cola_medicion = xQueueCreate(1, sizeof(medicion_t));   // antes de crear las tareas que la usan
    configASSERT(cola_medicion);
    // INICIALIZACIONES END
    ESP_ERROR_CHECK(red_wifi_init());
    xTaskCreate(micro_ros_task, "micro_ros_task", MICRO_ROS_TASK_STACK, NULL, MICRO_ROS_TASK_PRIO, NULL);
    if (laser_listo || us_listo) {   // con al menos uno andando ya se mide y se publica
        xTaskCreate(medicion_task, "medicion_task", MEDICION_TASK_STACK, NULL, MEDICION_TASK_PRIO, NULL);
    }
}
