#pragma once

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c_master.h"

#define MPU6050_WHO_AM_I_REG_ADDR 0x75   /*!< Register addresses of the "who am I" register */
#define MPU6050_PWR_MGMT_1_REG_ADDR 0x6B /*!< Register addresses of the power management register */

#define MPU6050_FIFO_R_W 0x74
#define MPU6050_FIFO_RESET 0x6A
#define MPU6050_FIFO_EN 0x23
#define MPU6050_FIFO_COUNT_H 0x72
#define MPU6050_FIFO_COUNT_L 0x73
#define MPU6050_CONFIG 0x1A
#define MPU6050_INT_ENABLE 0x38
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_GYRO_CONFIG 0x1B
#define SMPLRT_DIV 0x19

#define FIFO_BURST_LEN 12

typedef struct {
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
} mpu6050_data_t;

esp_err_t mpu6050_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len);

/**
 * @brief Write a byte to a MPU6050 sensor register
 */
esp_err_t mpu6050_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data);

/**
 * @brief i2c master initialization
 */
void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle);
