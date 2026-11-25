#pragma once
#include <stdint.h>
#include "esp_err.h"

// Resultado de una medición del láser
typedef enum {
    LASER_OK,           // medición válida (range status = 11)
    LASER_SIN_OBJETO,   // no vio nada (el sensor devuelve ~8190 mm)  -> en ROS: +Inf
    LASER_INVALIDA,     // el sensor la marcó como dudosa (señal débil, sigma, fase)
    LASER_ERROR_I2C,    // falló la comunicación                     -> en ROS: NaN
    LASER_TIMEOUT,      // el sensor no terminó a tiempo             -> en ROS: NaN
} laser_estado_t;

// Inicializa el bus I²C y el sensor (llamar una sola vez)
esp_err_t laser_init(void);

// Mide una vez (bloquea ~timing budget). Siempre escribe en *mm lo que leyó,
// pero solo es confiable si devuelve LASER_OK.
laser_estado_t laser_medir(uint16_t *mm);

// Tasa de señal de la última medición, en MCPS (cuánta luz volvió)
float laser_ultima_senal_mcps(void);

// Cambia el tiempo por medición (mínimo 20000 us). Más tiempo = menos ruido.
esp_err_t laser_set_budget_us(uint32_t us);

// Texto del estado, para imprimir
const char *laser_estado_str(laser_estado_t e);
