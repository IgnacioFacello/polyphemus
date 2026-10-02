#pragma once

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c_master.h"

#define MPU6050_WHO_AM_I_REG_ADDR 0x75   /*!< Register addresses of the "who am I" register */

esp_err_t mpu6050_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len);

/**
 * @brief Write a byte to a MPU6050 sensor register
 */
esp_err_t mpu6050_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data);

/**
 * @brief i2c master initialization
 */
void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle);