#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
//#include "encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#include "as5600.h"

#define I2C_MASTER_SCL_IO   22
#define I2C_MASTER_SDA_IO   21
#define I2C_MASTER_FREQ_HZ  400000
#define TIMEOUT_MS          100

#define AS5600_ADDR         0x36

#define ZPOS_ADDR           0x01
#define MPOS_ADDR           0x03
#define MANG_ADDR           0x05

#define CONF_ADDR_L         0x07
#define CONF_L_WD_MASK      0b00100000
#define CONF_L_FTH_MASK     0b00011100
#define CONF_L_SF_MASK      0b00000011

#define CONF_ADDR_R         0x08
#define CONF_R_PWMF_MASK    0b11000000
#define CONF_R_OUTS_MASK    0b00110000
#define CONF_R_HYST_MASK    0b00001100
#define CONF_R_PM_MASK      0b00000011

#define RAW_ANGLE_ADDR            0x0C
#define ANGLE_ADDR          0x0E

#define STATUS_ADDR         0x0B
#define AGC_ADDR            0x0B
#define MAG_ADDR            0x0B

static const char *TAG = "AS5600";

struct encoder_dev_t {
  i2c_master_bus_handle_t   bus;
  i2c_master_bus_config_t   bus_config;
  i2c_master_dev_handle_t   encoder;
  i2c_device_config_t       encoder_config;
  uint16_t                  start_position;
  uint16_t                  stop_position;
  uint16_t                  max_angle;
};

// =================== Aux Functions ===================

esp_err_t as5600_get_byte(
    encoder_handle_t handle,
    uint8_t reg_addr,
    uint8_t * data_buff
){
    return i2c_master_transmit_receive(
        handle->encoder,
        &reg_addr, 1,
        data_buff, 1,
        pdMS_TO_TICKS(100)
    );
}

esp_err_t write_byte(
    encoder_handle_t handle,
    uint8_t reg_addr,
    uint8_t data
){
    uint8_t write_buff[2] = { reg_addr, data };
    return i2c_master_transmit(
        handle->encoder,
        write_buff,
        sizeof(write_buff),
        pdMS_TO_TICKS(TIMEOUT_MS)
    );
}

esp_err_t as5600_get_two_bytes(
    encoder_handle_t handle,
    uint8_t reg_addr,
    uint16_t * data
){
    uint8_t aux[2];
    ESP_ERROR_CHECK(as5600_get_byte(handle, reg_addr+0, &aux[0]));
    ESP_ERROR_CHECK(as5600_get_byte(handle, reg_addr+1, &aux[1]));
    *data = (aux[0] << 8) + aux[1];
     return ESP_OK;
}


bool as5600_write_conf(encoder_handle_t handle, uint8_t addr, uint16_t data){
    esp_err_t err;
    err = write_byte(handle, addr+0, data & 0xFF);
    // xxxxxxxx WWWWWWWW
    if ( err != ESP_OK ) { return false; }
    err = write_byte(handle, addr+1, data >> 8);
    // WWWWWWWW xxxxxxxx
    if ( err != ESP_OK ) { return false; }
    return true;
}

// =================== Status ===================

void as5600_check_status(uint8_t status){
    bool md = status & (1 << 5);
    bool ml = status & (1 << 4);
    bool mh = status & (1 << 3);

    ESP_LOGI(TAG, "STATUS: 0x%02X | MD=%d ML=%d MH=%d", status, md, ml, mh);
    if (!md) {     ESP_LOGI(TAG, "No se detectó el iman");         }
    else if (ml) { ESP_LOGI(TAG, "Iman detectado pero muy lejos"); }
    else if (mh) { ESP_LOGI(TAG, "Iman detectado pero muy cerca"); }
}

esp_err_t as5600_get_status(encoder_handle_t handle, uint8_t * data_buf){
    return as5600_get_byte(handle, STATUS_ADDR, data_buf);
}

// =================== Outputs ===================

void as5600_get_raw_angle(encoder_handle_t handle, uint16_t* data){
    ESP_ERROR_CHECK(
        as5600_get_two_bytes(handle, RAW_ANGLE_ADDR, data)
    );
}

void as5600_get_angle(encoder_handle_t handle, uint16_t* data){
    ESP_ERROR_CHECK(
        as5600_get_two_bytes(handle, ANGLE_ADDR, data)
    );
}

// =================== Configuration ===================

bool set_start_pos(encoder_handle_t handle, uint16_t start_position){
    if (as5600_write_conf(handle, ZPOS_ADDR, start_position)) {
        handle->start_position = start_position;
        return true;
    } else {
        ESP_LOGE(TAG, "set_start_pos: failed to write zpos.");
        return false;
    }
}

bool set_stop_pos(encoder_handle_t handle, uint16_t start_position){
    if (as5600_write_conf(handle, MPOS_ADDR, start_position)) {
        handle->start_position = start_position;
        return true;
    } else {
        ESP_LOGE(TAG, "set_stop_pos: failed to write mpos.");
        return false;
    }
}

bool set_max_angle(encoder_handle_t handle, uint16_t start_position){
    if (as5600_write_conf(handle, MANG_ADDR, start_position)) {
        handle->start_position = start_position;
        return true;
    } else {
        ESP_LOGE(TAG, "set_max_angle: failed to write mang.");
        return false;
    }
}

esp_err_t create_i2c_bus(i2c_master_bus_config_t * bus_config, i2c_master_bus_handle_t * bus_handle){
    esp_err_t err;
    err = i2c_new_master_bus(bus_config, bus_handle);
    if (err == ESP_OK){
        ESP_LOGI(TAG, "Master bus created succesfully");
    }
    else {
        ESP_LOGE(TAG, "Failed on bus creation");
    }
    return err;
}

esp_err_t check_device_presence(i2c_master_bus_handle_t bus_handle){
    esp_err_t err;
    err = i2c_master_probe(bus_handle, AS5600_ADDR, pdMS_TO_TICKS(100));
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Encoder found correctly");
    } else {
        ESP_LOGE(TAG, "Sensor could not be found");
    }
    return err;
}

esp_err_t add_device_to_bus(
    i2c_master_bus_handle_t bus_handle,
    i2c_device_config_t * dev_config,
    i2c_master_dev_handle_t * dev_handle){
    esp_err_t err;
    err = i2c_master_bus_add_device(bus_handle, dev_config, dev_handle);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Device created correctly");
    } else {
        ESP_LOGE(TAG, "Failed to create device");
    }
    return err;
}


esp_err_t as5600_init(encoder_handle_t * handle)
{
    struct encoder_dev_t *enc = calloc(1, sizeof(struct encoder_dev_t));
    if (enc == NULL) {
        return ESP_ERR_NO_MEM;
    }

    i2c_master_bus_handle_t bus_handle;
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,                  //
        .sda_io_num = I2C_MASTER_SDA_IO,        // GPIO 21 por default
        .scl_io_num = I2C_MASTER_SCL_IO,        // GPIO 22 por default
        .clk_source = I2C_CLK_SRC_DEFAULT,      // Fuente del tick de reloj
        .glitch_ignore_cnt = 7,                 //
        .flags.enable_internal_pullup = false,
    };
    i2c_master_dev_handle_t dev_handle;
    i2c_device_config_t dev_config = {
        .device_address = AS5600_ADDR,          // Direccion del sensor
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,  // 7 son suficientes
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,     // 400khz
    };
    esp_err_t err;

    err = create_i2c_bus(&bus_config, &bus_handle);
    if (err != ESP_OK) { free(enc); return err; }

    err = check_device_presence(bus_handle);
    if (err != ESP_OK) { i2c_del_master_bus(bus_handle);
                        free(enc);
                        return err;
    }

    err = add_device_to_bus(bus_handle, &dev_config, &dev_handle);
    if (err != ESP_OK) { i2c_del_master_bus(bus_handle);
                        free(enc);
                        return err;
    }

    enc->bus_config = bus_config;
    enc->bus = bus_handle;
    enc->encoder_config = dev_config;
    enc->encoder = dev_handle;
    enc->start_position = 0;
    enc->stop_position = 0;
    enc->max_angle = 360;

    *handle = enc;
    return ESP_OK;
}

esp_err_t as5600_deinit(encoder_handle_t handle)
{
    esp_err_t err;
    err = i2c_master_bus_rm_device(handle->encoder);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove device from bus: %d", err);
        return err;
    }
    err = i2c_del_master_bus(handle->bus);
    free(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete bus: %d", err);
        return err;
    }
    return ESP_OK;
}
