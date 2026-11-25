/*
 * Driver del sensor ultrasonico HC-SR04 usando el periferico RMT del ESP32.
 * Pensado para ESP-IDF 5.4 (API nueva de RMT).
 *
 *   - Canal RMT TX -> pin TRIG (pulso de 10 us)
 *   - Canal RMT RX -> pin ECHO (mide el ancho del pulso de eco)
 *
 * ECHO sale a la tension de alimentacion: a 3,3 V va directo; a 5 V usar divisor (1k + 2k).
 */
#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Configuracion del sensor: solo hay que elegir los pines. */
typedef struct {
    gpio_num_t trig_gpio;   /*!< GPIO conectado a TRIG (salida del ESP32). GPIO_NUM_NC = este sensor comparte
                                 el TRIG de otro: solo el primero del grupo lo crea y su pulso dispara a todos */
    gpio_num_t echo_gpio;   /*!< GPIO conectado a ECHO (entrada del ESP32, con divisor resistivo) */
} hcsr04_config_t;

/** Configuracion por defecto: pines del menuconfig ("Ultrasonido (HC-SR04)"). */
#define HCSR04_CONFIG_DEFAULT() { \
    .trig_gpio = CONFIG_HCSR04_TRIG_PIN, \
    .echo_gpio = CONFIG_HCSR04_ECHO_PIN, \
}

typedef struct hcsr04_t *hcsr04_handle_t;

/**
 * @brief Crea los canales RMT (TX y RX) y deja el sensor listo para medir.
 *
 * @return ESP_OK si salio bien; otro codigo si fallo la reserva de canales, etc.
 */
esp_err_t hcsr04_new(const hcsr04_config_t *config, hcsr04_handle_t *out_handle);

/**
 * @brief Hace UNA medicion y devuelve la distancia en centimetros.
 *
 * Respeta internamente los 60 ms minimos entre disparos que pide el datasheet
 * (contados desde el ultimo disparo de CUALQUIER sensor, no solo de este).
 * La funcion bloquea hasta ~50 ms mientras espera el eco.
 *
 * @return ESP_OK                    medicion valida
 *         ESP_ERR_TIMEOUT           no llego ningun eco (sensor desconectado o mal cableado)
 *         ESP_ERR_INVALID_RESPONSE  eco fuera del rango 2 cm - 400 cm (sin objeto, muy cerca o muy lejos)
 */
esp_err_t hcsr04_measure_cm(hcsr04_handle_t handle, float *distance_cm);

/**
 * @brief Dispara VARIOS sensores a la vez y devuelve la distancia de cada uno.
 *
 * Usar solo con sensores que no se interfieran entre si. Si comparten TRIG, el grupo
 * debe incluir al sensor que lo creo (el que tiene trig_gpio) y a todos los que cuelgan de el. Respeta los 60 ms minimos
 * desde el ultimo disparo de cualquier sensor. Bloquea hasta ~50 ms.
 *
 * @param hs           handles de los sensores del grupo
 * @param n            cantidad de sensores
 * @param distance_cm  salida: distancia de cada sensor (valida solo si errs[i] == ESP_OK)
 * @param errs         salida: resultado de cada sensor, con los mismos codigos que hcsr04_measure_cm()
 * @return ESP_OK, o ESP_ERR_INVALID_ARG si algun puntero es nulo
 */
esp_err_t hcsr04_measure_group_cm(const hcsr04_handle_t *hs, size_t n,
                                  float *distance_cm, esp_err_t *errs);

/**
 * @brief Medicion en dos partes, para hacer otra cosa mientras vuela el sonido
 *        (ej: medir con el laser al mismo tiempo).
 *
 *   hcsr04_start(h);                  // dispara y vuelve enseguida
 *   ...otra cosa...                   // el RMT captura el eco solo, por hardware
 *   hcsr04_wait_cm(h, &cm, &eco_us);  // espera el eco (si no llego todavia) y lo convierte
 *
 * hcsr04_start() respeta los 60 ms minimos entre disparos (puede esperar).
 * hcsr04_wait_cm() devuelve los mismos codigos que hcsr04_measure_cm().
 * echo_us (opcional, puede ser NULL): ancho del eco en us = tiempo de ida y vuelta del sonido.
 */
esp_err_t hcsr04_start(hcsr04_handle_t handle);
esp_err_t hcsr04_wait_cm(hcsr04_handle_t handle, float *distance_cm, uint32_t *echo_us);

/** Libera los canales RMT y la memoria. */
esp_err_t hcsr04_del(hcsr04_handle_t handle);

#ifdef __cplusplus
}
#endif
