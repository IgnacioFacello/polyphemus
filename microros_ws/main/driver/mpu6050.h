#pragma once

#include <stdint.h>
#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

#define MPU6050_WHO_AM_I_REG_ADDR 0x75   /*!< Register addresses of the "who am I" register */
#define MPU6050_PWR_MGMT_1_REG_ADDR 0x6B /*!< Register addresses of the power management register */

#define MPU6050_FIFO_R_W 0x74
#define MPU6050_FIFO_EN 0x23
#define MPU6050_FIFO_COUNT_H 0x72
#define MPU6050_FIFO_COUNT_L 0x73
#define MPU6050_CONFIG 0x1A
#define MPU6050_INT_ENABLE 0x38
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_GYRO_CONFIG 0x1B
#define SMPLRT_DIV 0x19
#define MPU6050_INT_PIN_CFG 0x37

#define MPU6050_USER_CTRL     0x6A  
#define DMP_PACKET_SIZE       42
#define GYRO_LSB_PER_DPS_DMP  16.4f                /* ±2000 °/s */
#define ACCEL_LSB_PER_G_DMP   8192.0f            /* ±2 g */
#define Q30                   1073741824.0f

typedef struct {
	float ax, ay, az;
	float gx, gy, gz;
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

esp_err_t mpu6050_dmp_prepare(i2c_master_dev_handle_t dev_handle);
esp_err_t mpu6050_dmp_start(i2c_master_dev_handle_t dev_handle);
esp_err_t mpu6050_dmp_read_fifo_count(i2c_master_dev_handle_t dev_handle, uint16_t *count);
esp_err_t mpu6050_dmp_read_fifo_packet(i2c_master_dev_handle_t dev_handle, uint8_t packet[DMP_PACKET_SIZE]);
esp_err_t mpu6050_dmp_reset_fifo(i2c_master_dev_handle_t dev_handle);
