/*
 * encoder.h — AS5600 encoder driver built on the ESP-IDF I2C peripheral.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

typedef struct encoder_dev_t *encoder_handle_t;


// =================== Status ===================
esp_err_t as5600_get_status(encoder_handle_t handle, uint8_t * data_buf);

/* @brief
 * Imprime por consola el status actual del sensor
 */
void as5600_check_status(uint8_t status);

// =================== Outputs ===================
void as5600_get_raw_angle(encoder_handle_t handle, uint16_t* data);

void as5600_get_angle(encoder_handle_t handle, uint16_t* data);

// =================== Configuration ===================

/*
 * @brief
 * Configura la posición de inicio del encoder
 * Este valor será el nuevo 0
 */
bool set_start_pos(encoder_handle_t handle, uint16_t start_position);

/*
 * @brief
 * Configura la posición maxima del encoder del encoder
 * esta posición será el nuevo 4095
 */
bool set_stop_pos(encoder_handle_t handle, uint16_t start_position);


/*
 * @brief
 * Configura el angulo maximo del encoder
 * El rango 0-4095 será mapeado a 0-max_angle
 */
bool set_max_angle(encoder_handle_t handle, uint16_t start_position);
