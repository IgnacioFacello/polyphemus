/*
 * encoder.h — AS5600 encoder driver built on the ESP-IDF I2C peripheral.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "encoder.h"
#include "esp_err.h"

typedef struct encoder_dev_t *encoder_handle_t;

typedef struct {
    uint8_t  i2c_port;                  // I2C port number
    uint8_t  sda_io_num;                // GPIO number for SDA
    uint8_t  scl_io_num;                // GPIO number for SCL
    uint32_t clk_source;                // Clock source for I2C
    uint32_t glitch_ignore_cnt;         // Glitch filter count
    bool     enable_internal_pullup;    // Enable internal pull-up resistors
} encoder_config_t;

#define ENCODER_DEFAULT_CONFIG()            \
    {                                       \
        .i2c_port   = I2C_NUM_0;            \
        .sda_io_num = I2C_MASTER_SDA_IO;    \
        .scl_io_num = I2C_MASTER_SCL_IO;    \
        .clk_source = I2C_CLK_SRC_DEFAULT;  \
        .glitch_ignore_cnt = 7;             \
        .enable_internal_pullup = false;    \
    };

esp_err_t as5600_init(encoder_handle_t * handle);

esp_err_t as5600_deinit(encoder_handle_t handle);

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
