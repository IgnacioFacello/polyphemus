#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include <string.h>
#include "driver/i2c_master.h"
#include "driver/mpu6050.h"
#include "dmp_firmware.h"

static const char *TAG = "mpu6050";

#define DMP_REG_XG_OFFS_TC    0x00
#define DMP_REG_SMPLRT_DIV    0x19
#define DMP_REG_CONFIG        0x1A
#define DMP_REG_GYRO_CONFIG   0x1B
#define DMP_REG_MOT_THR       0x1F
#define DMP_REG_MOT_DUR       0x20
#define DMP_REG_ZRMOT_THR     0x21
#define DMP_REG_ZRMOT_DUR     0x22
#define DMP_REG_I2C_SLV0_ADDR 0x25
#define DMP_REG_INT_ENABLE    0x38
#define DMP_REG_INT_STATUS    0x3A
#define DMP_REG_USER_CTRL     0x6A
#define DMP_REG_BANK_SEL      0x6D
#define DMP_REG_MEM_START     0x6E
#define DMP_REG_MEM_RW        0x6F
#define DMP_REG_DMP_CFG_1     0x70
#define DMP_REG_DMP_CFG_2     0x71
#define DMP_REG_FIFO_COUNT_H  0x72
#define DMP_REG_FIFO_RW       0x74

#ifndef DMP_FIFO_RATE_DIVISOR
#define DMP_FIFO_RATE_DIVISOR 0x00
#endif

#define I2C_MASTER_SCL_IO 22       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 21        /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0                       /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                    /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                    /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

#define MPU6050_SENSOR_ADDR 0x68         /*!< Address of the MPU6050 sensor */
#define MPU6050_PWR_MGMT_1_REG_ADDR 0x6B /*!< Register addresses of the power management register */
#define MPU6050_RESET_BIT 7

/**
 * @brief Read a sequence of bytes from a MPU6050 sensor registers
 */
esp_err_t mpu6050_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Write a byte to a MPU6050 sensor register
 */
esp_err_t mpu6050_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t mpu_write_reg(i2c_master_dev_handle_t dev_handle, uint8_t reg, uint8_t data)
{
    return mpu6050_register_write_byte(dev_handle, reg, data);
}

static esp_err_t mpu_read_reg(i2c_master_dev_handle_t dev_handle, uint8_t reg, uint8_t *data, size_t len)
{
    return mpu6050_register_read(dev_handle, reg, data, len);
}

static esp_err_t dmp_write_mem(i2c_master_dev_handle_t dev_handle, uint8_t bank, uint8_t addr, const uint8_t *data, size_t len)
{
    uint8_t verify[16];
    size_t i = 0;

    while (i < len) {
        size_t chunk = (len - i < 16) ? (len - i) : 16;
        if (chunk > 256u - addr) chunk = 256u - addr;

        ESP_RETURN_ON_ERROR(mpu_write_reg(dev_handle, DMP_REG_BANK_SEL, bank), TAG, "bank");
        ESP_RETURN_ON_ERROR(mpu_write_reg(dev_handle, DMP_REG_MEM_START, addr), TAG, "addr");
        for (size_t j = 0; j < chunk; j++) {
            ESP_RETURN_ON_ERROR(mpu_write_reg(dev_handle, DMP_REG_MEM_RW, data[i + j]), TAG, "mem write");
        }

        ESP_RETURN_ON_ERROR(mpu_write_reg(dev_handle, DMP_REG_BANK_SEL, bank), TAG, "bank");
        ESP_RETURN_ON_ERROR(mpu_write_reg(dev_handle, DMP_REG_MEM_START, addr), TAG, "addr");
        ESP_RETURN_ON_ERROR(mpu_read_reg(dev_handle, DMP_REG_MEM_RW, verify, chunk), TAG, "mem read");
        if (memcmp(verify, data + i, chunk) != 0) {
            ESP_LOGE(TAG, "DMP memory verify failed (bank %u, addr 0x%02X)", bank, addr);
            return ESP_FAIL;
        }

        i += chunk;
        addr = (uint8_t)(addr + chunk);
        if (addr == 0) bank++;
    }
    return ESP_OK;
}

static esp_err_t mpu_dmp_init(i2c_master_dev_handle_t dev_handle)
{
    uint8_t value = 0;

    ESP_RETURN_ON_ERROR(mpu_write_reg(dev_handle, MPU6050_PWR_MGMT_1_REG_ADDR, 0x80), TAG, "reset");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(mpu_write_reg(dev_handle, MPU6050_PWR_MGMT_1_REG_ADDR, 0x03), TAG, "wake");
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_RETURN_ON_ERROR(mpu_write_reg(dev_handle, DMP_REG_I2C_SLV0_ADDR, 0x7F), TAG, "slv0");
    ESP_RETURN_ON_ERROR(mpu_write_reg(dev_handle, DMP_REG_USER_CTRL, 0x00), TAG, "i2c master off");
    ESP_RETURN_ON_ERROR(mpu_write_reg(dev_handle, DMP_REG_I2C_SLV0_ADDR, 0x68), TAG, "slv0 self");
    ESP_RETURN_ON_ERROR(mpu_write_reg(dev_handle, DMP_REG_USER_CTRL, 0x02), TAG, "i2c master reset");
    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t smplrt_div = 1000 / CONFIG_SAMPLE_RATE - 1;
    ESP_RETURN_ON_ERROR(mpu_write_reg(dev_handle, DMP_REG_SMPLRT_DIV, smplrt_div), TAG, "rate");
    ESP_RETURN_ON_ERROR(mpu_write_reg(dev_handle, DMP_REG_CONFIG, 0x0B), TAG, "config");
    ESP_RETURN_ON_ERROR(mpu_write_reg(dev_handle, DMP_REG_GYRO_CONFIG, 0x18), TAG, "gyro cfg");

    ESP_LOGI(TAG, "Loading DMP firmware (%d bytes)...", DMP_FIRMWARE_SIZE);
    ESP_RETURN_ON_ERROR(dmp_write_mem(dev_handle, 0x00, 0x00, dmp_firmware, DMP_FIRMWARE_SIZE), TAG, "firmware");

    const uint8_t rate_update[2] = {0x00, DMP_FIFO_RATE_DIVISOR};
    ESP_RETURN_ON_ERROR(dmp_write_mem(dev_handle, 0x02, 0x16, rate_update, sizeof(rate_update)), TAG, "fifo rate");

    ESP_RETURN_ON_ERROR(mpu_write_reg(dev_handle, DMP_REG_DMP_CFG_1, 0x03), TAG, "dmp cfg1");
    ESP_RETURN_ON_ERROR(mpu_write_reg(dev_handle, DMP_REG_DMP_CFG_2, 0x00), TAG, "dmp cfg2");

    ESP_RETURN_ON_ERROR(mpu_read_reg(dev_handle, DMP_REG_XG_OFFS_TC, &value, 1), TAG, "otp read");
    ESP_RETURN_ON_ERROR(mpu_write_reg(dev_handle, DMP_REG_XG_OFFS_TC, value & ~0x01), TAG, "otp clear");

    ESP_RETURN_ON_ERROR(mpu_write_reg(dev_handle, DMP_REG_MOT_THR, 2), TAG, "mot thr");
    ESP_RETURN_ON_ERROR(mpu_write_reg(dev_handle, DMP_REG_ZRMOT_THR, 156), TAG, "zrmot thr");
    ESP_RETURN_ON_ERROR(mpu_write_reg(dev_handle, DMP_REG_MOT_DUR, 80), TAG, "mot dur");
    ESP_RETURN_ON_ERROR(mpu_write_reg(dev_handle, DMP_REG_ZRMOT_DUR, 0), TAG, "zrmot dur");

    ESP_RETURN_ON_ERROR(mpu_write_reg(dev_handle, DMP_REG_USER_CTRL, 0x40), TAG, "fifo en");
    ESP_RETURN_ON_ERROR(mpu_write_reg(dev_handle, DMP_REG_USER_CTRL, 0x48), TAG, "dmp reset");
    vTaskDelay(pdMS_TO_TICKS(5));
    ESP_RETURN_ON_ERROR(mpu_write_reg(dev_handle, DMP_REG_USER_CTRL, 0x40), TAG, "dmp off");
    ESP_RETURN_ON_ERROR(mpu_write_reg(dev_handle, DMP_REG_USER_CTRL, 0x44), TAG, "fifo reset");
    ESP_RETURN_ON_ERROR(mpu_read_reg(dev_handle, DMP_REG_INT_STATUS, &value, 1), TAG, "int status");

    return ESP_OK;
}

esp_err_t mpu6050_dmp_prepare(i2c_master_dev_handle_t dev_handle)
{
    ESP_RETURN_ON_FALSE(dev_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid device handle");
    ESP_RETURN_ON_ERROR(mpu_dmp_init(dev_handle), TAG, "DMP init");
    ESP_RETURN_ON_ERROR(mpu_write_reg(dev_handle, MPU6050_INT_PIN_CFG, 0x10), TAG, "INT_PIN_CFG");
    ESP_LOGI(TAG, "DMP loaded");
    return ESP_OK;
}

esp_err_t mpu6050_dmp_start(i2c_master_dev_handle_t dev_handle)
{
    ESP_RETURN_ON_FALSE(dev_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid device handle");
    ESP_RETURN_ON_ERROR(mpu_write_reg(dev_handle, DMP_REG_INT_ENABLE, 0x12), TAG, "INT_ENABLE");
    return mpu_write_reg(dev_handle, DMP_REG_USER_CTRL, 0xC0);
}

esp_err_t mpu6050_dmp_read_fifo_count(i2c_master_dev_handle_t dev_handle, uint16_t *count)
{
    uint8_t buffer[2];
    ESP_RETURN_ON_FALSE(dev_handle != NULL && count != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid FIFO count arguments");
    ESP_RETURN_ON_ERROR(mpu_read_reg(dev_handle, DMP_REG_FIFO_COUNT_H, buffer, sizeof(buffer)), TAG, "FIFO count");
    *count = ((uint16_t)buffer[0] << 8) | buffer[1];
    return ESP_OK;
}

esp_err_t mpu6050_dmp_read_fifo_packet(i2c_master_dev_handle_t dev_handle, uint8_t packet[DMP_PACKET_SIZE])
{
    ESP_RETURN_ON_FALSE(dev_handle != NULL && packet != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid FIFO packet arguments");
    return mpu_read_reg(dev_handle, DMP_REG_FIFO_RW, packet, DMP_PACKET_SIZE);
}

esp_err_t mpu6050_dmp_reset_fifo(i2c_master_dev_handle_t dev_handle)
{
    ESP_RETURN_ON_FALSE(dev_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid device handle");
    return mpu_write_reg(dev_handle, DMP_REG_USER_CTRL, 0xC4);
}

/**
 * @brief i2c master initialization
 */
void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    static i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}
