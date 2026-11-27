/*
 * Driver mínimo del VL53L0X (ToF láser) para ESP-IDF 5.x.
 *
 * La secuencia de inicialización, la tabla de tuning y el cálculo del
 * timing budget están adaptados de la librería de Pololu
 * (https://github.com/pololu/vl53l0x-arduino), que a su vez se basa en
 * la API oficial de ST (STSW-IMG005).
 */
#include "vl53l0x_drv.h"
#include <assert.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"           // vTaskDelay
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"               // reloj del ESP32, para no esperar al sensor para siempre

#define I2C_SDA_IO     21 //GPIO 21 y 22 son los pines I²C por defecto del ESP32
#define I2C_SCL_IO     22
#define I2C_FREQ_HZ    400000 //el datasheet dice que el VL53L0X soporta hasta Fast Mode
#define LASER_ADDR     0x29 //es la dirección de 7 bits.
//el datasheet muestra 0x52 y 0x53 porque incluye el bit de lectura/escritura (0x29 << 1 = 0x52)
#define WHO_AM_I_REG   0xC0 //registro de identificación del modelo
#define WHO_AM_I_VAL   0xEE
#define TIMEOUT_MS     100  //cuánto esperamos una transacción I²C

#define POLL_TIMEOUT_MS    500   // cuánto esperamos como máximo a que el sensor termine algo (medir, calibrar)
#define FUERA_DE_RANGO_MM  8190  // si el sensor no ve nada, devuelve ~8190 mm
#define RANGE_STATUS_OK    11    // en el byte de 0x14: (status & 0x78) >> 3 == 11 -> medición válida
#define MIN_BUDGET_US      20000 // mínimo timing budget que acepta la API de ST

// Nombres de los registros que usamos
#define SYSRANGE_START                              0x00  // escribir 0x01 acá = "medí"
#define SYSTEM_SEQUENCE_CONFIG                      0x01
#define SYSTEM_INTERRUPT_CONFIG_GPIO                0x0A
#define SYSTEM_INTERRUPT_CLEAR                      0x0B  // escribir 0x01 acá = "limpiar"
#define RESULT_INTERRUPT_STATUS                     0x13  // dice si la medición está lista
#define RESULT_RANGE_STATUS                         0x14  // bloque de 12 bytes: status, señal, distancia (en +10)
#define FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44
#define MSRC_CONFIG_TIMEOUT_MACROP                  0x46
#define PRE_RANGE_CONFIG_VCSEL_PERIOD               0x50
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          0x51
#define MSRC_CONFIG_CONTROL                         0x60
#define FINAL_RANGE_CONFIG_VCSEL_PERIOD             0x70
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        0x71
#define GPIO_HV_MUX_ACTIVE_HIGH                     0x84
#define VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           0x89
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_0            0xB0
#define GLOBAL_CONFIG_REF_EN_START_SELECT           0xB6
#define DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         0x4E
#define DYNAMIC_SPAD_REF_EN_START_OFFSET            0x4F

static const char *TAG = "LASER";

//variables internas
static i2c_master_bus_handle_t bus = NULL;
static i2c_master_dev_handle_t dev = NULL;   // el "contacto" del sensor en el bus
static uint8_t  stop_variable = 0;           // dato que lee el init y se usa en cada medición
static uint32_t timing_budget_us = 0;        // cuánto tarda cada medición (~33 ms por defecto)
static uint16_t ultima_senal = 0;            // tasa de señal de la última medición (punto fijo 9.7)

// Primer error de I²C desde la última vez que se puso en ESP_OK.
// Los helpers de abajo lo anotan, así no hace falta chequear cada línea:
// se pone en ESP_OK, se hacen varias lecturas/escrituras y al final se mira.
static esp_err_t i2c_err = ESP_OK;


/* ====================================================================
 * Funciones para leer y escribir registros (diapos 16 y 17)
 * ==================================================================== */

static void anotar_error(esp_err_t err)
{
    if (err != ESP_OK && i2c_err == ESP_OK) i2c_err = err;
}

// Escribir 1 byte en un registro: mandamos [registro, dato]
static void write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    anotar_error(i2c_master_transmit(dev, buf, 2, TIMEOUT_MS));
}

// Escribir 2 bytes (primero el más significativo)
static void write_reg16(uint8_t reg, uint16_t value)
{
    uint8_t buf[3] = { reg, (uint8_t)(value >> 8), (uint8_t)value };
    anotar_error(i2c_master_transmit(dev, buf, 3, TIMEOUT_MS));
}

// Escribir varios bytes seguidos (el sensor avanza de registro solo)
static void write_multi(uint8_t reg, const uint8_t *src, uint8_t n)
{
    uint8_t buf[8];
    assert(n <= sizeof(buf) - 1);
    buf[0] = reg;
    for (uint8_t i = 0; i < n; i++) buf[i + 1] = src[i];
    anotar_error(i2c_master_transmit(dev, buf, n + 1, TIMEOUT_MS));
}

// Leer 1 byte de un registro (igual que el who am I)
static uint8_t read_reg(uint8_t reg)
{
    uint8_t val = 0;
    anotar_error(i2c_master_transmit_receive(dev, &reg, 1, &val, 1, TIMEOUT_MS));
    return val;
}

// Leer 2 bytes y juntarlos: (MSB << 8) | LSB
static uint16_t read_reg16(uint8_t reg)
{
    uint8_t b[2] = { 0 };
    anotar_error(i2c_master_transmit_receive(dev, &reg, 1, b, 2, TIMEOUT_MS));
    return ((uint16_t)b[0] << 8) | b[1];
}

// Leer varios bytes seguidos
static void read_multi(uint8_t reg, uint8_t *dst, uint8_t n)
{
    anotar_error(i2c_master_transmit_receive(dev, &reg, 1, dst, n, TIMEOUT_MS));
}

// Hora actual en milisegundos (para los timeouts)
static int64_t ahora_ms(void) { return esp_timer_get_time() / 1000; }


/* ====================================================================
 * Cuentas de tiempos internos del sensor (de Pololu / API de ST).
 * El sensor guarda sus tiempos en "macro períodos"; esto los pasa a
 * microsegundos y calcula/ajusta el timing budget.
 * ==================================================================== */

#define decode_vcsel_period(r)   ((((uint8_t)(r)) + 1) << 1)
#define calc_macro_period(p)     ((((uint32_t)2304 * (p) * 1655) + 500) / 1000)

typedef struct { bool tcc, msrc, dss, pre_range, final_range; } seq_enables_t;
typedef struct {
    uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;
    uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
    uint32_t msrc_dss_tcc_us, pre_range_us, final_range_us;
} seq_timeouts_t;

static uint16_t decode_timeout(uint16_t v)
{
    return (uint16_t)((v & 0x00FF) << (uint16_t)((v & 0xFF00) >> 8)) + 1;
}

static uint16_t encode_timeout(uint32_t mclks)
{
    if (mclks == 0) return 0;
    uint32_t ls = mclks - 1;
    uint16_t ms = 0;
    while ((ls & 0xFFFFFF00) > 0) { ls >>= 1; ms++; }
    return (ms << 8) | (ls & 0xFF);
}

static uint32_t mclks_to_us(uint16_t mclks, uint8_t vcsel_pclks)
{
    uint32_t macro_ns = calc_macro_period(vcsel_pclks);
    return ((mclks * macro_ns) + 500) / 1000;
}

static uint32_t us_to_mclks(uint32_t us, uint8_t vcsel_pclks)
{
    uint32_t macro_ns = calc_macro_period(vcsel_pclks);
    return (((us * 1000) + (macro_ns / 2)) / macro_ns);
}

static void get_seq_enables(seq_enables_t *e)
{
    uint8_t c = read_reg(SYSTEM_SEQUENCE_CONFIG);
    e->tcc         = (c >> 4) & 1;
    e->dss         = (c >> 3) & 1;
    e->msrc        = (c >> 2) & 1;
    e->pre_range   = (c >> 6) & 1;
    e->final_range = (c >> 7) & 1;
}

static void get_seq_timeouts(const seq_enables_t *e, seq_timeouts_t *t)
{
    t->pre_range_vcsel_period_pclks = decode_vcsel_period(read_reg(PRE_RANGE_CONFIG_VCSEL_PERIOD));
    t->msrc_dss_tcc_mclks = read_reg(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
    t->msrc_dss_tcc_us = mclks_to_us(t->msrc_dss_tcc_mclks, t->pre_range_vcsel_period_pclks);
    t->pre_range_mclks = decode_timeout(read_reg16(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    t->pre_range_us = mclks_to_us(t->pre_range_mclks, t->pre_range_vcsel_period_pclks);

    t->final_range_vcsel_period_pclks = decode_vcsel_period(read_reg(FINAL_RANGE_CONFIG_VCSEL_PERIOD));
    t->final_range_mclks = decode_timeout(read_reg16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    if (e->pre_range) t->final_range_mclks -= t->pre_range_mclks;
    t->final_range_us = mclks_to_us(t->final_range_mclks, t->final_range_vcsel_period_pclks);
}

// Costos fijos de cada etapa de la medición, en us (valores de la API de ST)
#define START_OVH 1910
#define END_OVH    960
#define MSRC_OVH   660
#define TCC_OVH    590
#define DSS_OVH    690
#define PRE_OVH    660
#define FINAL_OVH  550

static uint32_t get_timing_budget(void)
{
    seq_enables_t e; seq_timeouts_t t;
    get_seq_enables(&e);
    get_seq_timeouts(&e, &t);

    uint32_t b = START_OVH + END_OVH;
    if (e.tcc)         b += t.msrc_dss_tcc_us + TCC_OVH;
    if (e.dss)         b += 2 * (t.msrc_dss_tcc_us + DSS_OVH);
    else if (e.msrc)   b += t.msrc_dss_tcc_us + MSRC_OVH;
    if (e.pre_range)   b += t.pre_range_us + PRE_OVH;
    if (e.final_range) b += t.final_range_us + FINAL_OVH;
    return b;
}

static bool set_timing_budget(uint32_t budget_us)
{
    if (budget_us < MIN_BUDGET_US) return false;

    seq_enables_t e; seq_timeouts_t t;
    get_seq_enables(&e);
    get_seq_timeouts(&e, &t);

    uint32_t used = START_OVH + END_OVH;
    if (e.tcc)         used += t.msrc_dss_tcc_us + TCC_OVH;
    if (e.dss)         used += 2 * (t.msrc_dss_tcc_us + DSS_OVH);
    else if (e.msrc)   used += t.msrc_dss_tcc_us + MSRC_OVH;
    if (e.pre_range)   used += t.pre_range_us + PRE_OVH;

    if (e.final_range) {
        used += FINAL_OVH;
        if (used > budget_us) return false;
        uint32_t final_mclks = us_to_mclks(budget_us - used, t.final_range_vcsel_period_pclks);
        if (e.pre_range) final_mclks += t.pre_range_mclks;
        write_reg16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encode_timeout(final_mclks));
        timing_budget_us = budget_us;
    }
    return true;
}


/* ====================================================================
 * Pasos de la "afinación" (de Pololu / API de ST)
 * ==================================================================== */

// Lee de la memoria del chip cuántos SPADs (detectores) usar y de qué tipo
static bool get_spad_info(uint8_t *count, bool *is_aperture)
{
    write_reg(0x80, 0x01);
    write_reg(0xFF, 0x01);
    write_reg(0x00, 0x00);

    write_reg(0xFF, 0x06);
    write_reg(0x83, read_reg(0x83) | 0x04);
    write_reg(0xFF, 0x07);
    write_reg(0x81, 0x01);
    write_reg(0x80, 0x01);
    write_reg(0x94, 0x6B);
    write_reg(0x83, 0x00);

    int64_t t0 = ahora_ms();
    while (read_reg(0x83) == 0x00) {
        if (i2c_err != ESP_OK || ahora_ms() - t0 > POLL_TIMEOUT_MS) return false;
    }
    write_reg(0x83, 0x01);
    uint8_t tmp = read_reg(0x92);
    *count = tmp & 0x7F;
    *is_aperture = (tmp >> 7) & 0x01;

    write_reg(0x81, 0x00);
    write_reg(0xFF, 0x06);
    write_reg(0x83, read_reg(0x83) & ~0x04);
    write_reg(0xFF, 0x01);
    write_reg(0x00, 0x01);
    write_reg(0xFF, 0x00);
    write_reg(0x80, 0x00);
    return i2c_err == ESP_OK;
}

// Una calibración interna: arranca, espera a que termine y limpia
static bool single_ref_calibration(uint8_t vhv_init_byte)
{
    write_reg(SYSRANGE_START, 0x01 | vhv_init_byte);
    int64_t t0 = ahora_ms();
    while ((read_reg(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
        if (i2c_err != ESP_OK || ahora_ms() - t0 > POLL_TIMEOUT_MS) return false;
    }
    write_reg(SYSTEM_INTERRUPT_CLEAR, 0x01);
    write_reg(SYSRANGE_START, 0x00);
    return i2c_err == ESP_OK;
}

// Ajustes de fábrica de ST: lista de pares (registro, valor)
static const uint8_t TUNING[][2] = {
    {0xFF,0x01},{0x00,0x00},{0xFF,0x00},{0x09,0x00},{0x10,0x00},{0x11,0x00},
    {0x24,0x01},{0x25,0xFF},{0x75,0x00},{0xFF,0x01},{0x4E,0x2C},{0x48,0x00},
    {0x30,0x20},{0xFF,0x00},{0x30,0x09},{0x54,0x00},{0x31,0x04},{0x32,0x03},
    {0x40,0x83},{0x46,0x25},{0x60,0x00},{0x27,0x00},{0x50,0x06},{0x51,0x00},
    {0x52,0x96},{0x56,0x08},{0x57,0x30},{0x61,0x00},{0x62,0x00},{0x64,0x00},
    {0x65,0x00},{0x66,0xA0},{0xFF,0x01},{0x22,0x32},{0x47,0x14},{0x49,0xFF},
    {0x4A,0x00},{0xFF,0x00},{0x7A,0x0A},{0x7B,0x00},{0x78,0x21},{0xFF,0x01},
    {0x23,0x34},{0x42,0x00},{0x44,0xFF},{0x45,0x26},{0x46,0x05},{0x40,0x40},
    {0x0E,0x06},{0x20,0x1A},{0x43,0x40},{0xFF,0x00},{0x34,0x03},{0x35,0x44},
    {0xFF,0x01},{0x31,0x04},{0x4B,0x09},{0x4C,0x05},{0x4D,0x04},{0xFF,0x00},
    {0x44,0x00},{0x45,0x20},{0x47,0x08},{0x48,0x28},{0x67,0x00},{0x70,0x04},
    {0x71,0x01},{0x72,0xFE},{0x76,0x00},{0x77,0x00},{0xFF,0x01},{0x0D,0x01},
    {0xFF,0x00},{0x80,0x01},{0x01,0xF8},{0xFF,0x01},{0x8E,0x01},{0x00,0x01},
    {0xFF,0x00},{0x80,0x00},
};


/* ====================================================================
 * Bus, who am I e inicialización
 * ==================================================================== */

//I²C
static esp_err_t crear_bus(void)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_IO,
        .scl_io_num = I2C_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7, //filtro de ruido: si aparece un pulso muy cortito por interferencia, lo ignora.
        .flags.enable_internal_pullup = false, // el módulo violeta ya trae sus pull-ups, no hacen falta las del ESP32
    };
    esp_err_t err = i2c_new_master_bus(&cfg, &bus);
    if (err == ESP_OK) ESP_LOGI(TAG, "Bus I2C creado");
    else               ESP_LOGE(TAG, "No se pudo crear el bus I2C");
    return err;
}

//comprobar que el sensor está y es el correcto
static esp_err_t chequear_sensor(void)
{
    // 1) probe: hay alguien conectado en la dirección 0x29?
    if (i2c_master_probe(bus, LASER_ADDR, TIMEOUT_MS) != ESP_OK) {
        ESP_LOGE(TAG, "No hay nada en 0x%02X. Revisar cableado.", LASER_ADDR);
        return ESP_FAIL;
    }

    // 2) who am I: es posta un VL53L0X y no otro chip?
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LASER_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    // Si falla, devolvemos el error en vez de reiniciar el ESP32 (ESP_ERROR_CHECK)
    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo agregar el sensor al bus: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t reg = WHO_AM_I_REG, val = 0;
    err = i2c_master_transmit_receive(dev, &reg, 1, &val, 1, TIMEOUT_MS);

    //Si la comunicación falló o el valor no es 0xEE tira error y corta
    if (err != ESP_OK || val != WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "Who am I falló: 0xC0 = 0x%02X (esperado 0xEE)", val);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "0xC0 = 0x%02X -> VL53L0X encontrado", val);
    return ESP_OK;
}

//laser_init: inicializa el bus I²C y el sensor, lo deja listo para medir
esp_err_t laser_init(void)
{
    if (crear_bus() != ESP_OK)       return ESP_FAIL;
    if (chequear_sensor() != ESP_OK) return ESP_FAIL;

    i2c_err = ESP_OK;

    // a) Configuración básica: pines a 2,8 V, I2C estándar y leer la stop variable
    write_reg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
              read_reg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01);
    write_reg(0x88, 0x00);
    write_reg(0x80, 0x01);
    write_reg(0xFF, 0x01);
    write_reg(0x00, 0x00);
    stop_variable = read_reg(0x91);
    write_reg(0x00, 0x01);
    write_reg(0xFF, 0x00);
    write_reg(0x80, 0x00);
    write_reg(MSRC_CONFIG_CONTROL, read_reg(MSRC_CONFIG_CONTROL) | 0x12);
    write_reg16(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, (uint16_t)(0.25f * (1 << 7)));
    write_reg(SYSTEM_SEQUENCE_CONFIG, 0xFF);

    // b) Elegir los mejores SPADs (detectores de luz)
    uint8_t spad_count;
    bool spad_is_aperture;
    if (!get_spad_info(&spad_count, &spad_is_aperture)) {
        ESP_LOGE(TAG, "Error leyendo info de SPADs");
        return ESP_FAIL;
    }
    uint8_t spad_map[6];
    read_multi(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, spad_map, 6);
    write_reg(0xFF, 0x01);
    write_reg(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    write_reg(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    write_reg(0xFF, 0x00);
    write_reg(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);
    uint8_t first = spad_is_aperture ? 12 : 0;
    uint8_t enabled = 0;
    for (uint8_t i = 0; i < 48; i++) {
        if (i < first || enabled == spad_count) {
            spad_map[i / 8] &= ~(1 << (i % 8));
        } else if ((spad_map[i / 8] >> (i % 8)) & 0x1) {
            enabled++;
        }
    }
    write_multi(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, spad_map, 6);

    // c) Cargar los ajustes de fábrica de ST
    for (size_t i = 0; i < sizeof(TUNING) / sizeof(TUNING[0]); i++) {
        write_reg(TUNING[i][0], TUNING[i][1]);
    }

    // d) Avisar "medición lista" por el registro de interrupción
    write_reg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    write_reg(GPIO_HV_MUX_ACTIVE_HIGH, read_reg(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10);
    write_reg(SYSTEM_INTERRUPT_CLEAR, 0x01);

    // e) Tiempo por medición (perfil por defecto, ~33 ms)
    timing_budget_us = get_timing_budget();
    write_reg(SYSTEM_SEQUENCE_CONFIG, 0xE8);
    set_timing_budget(timing_budget_us);

    // f) Calibrarse a sí mismo (VHV y fase)
    write_reg(SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (!single_ref_calibration(0x40)) { ESP_LOGE(TAG, "Falló calibración VHV");  return ESP_FAIL; }
    write_reg(SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (!single_ref_calibration(0x00)) { ESP_LOGE(TAG, "Falló calibración fase"); return ESP_FAIL; }
    write_reg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

    // ¿Falló alguna lectura/escritura en el camino?
    if (i2c_err != ESP_OK) {
        ESP_LOGE(TAG, "Error de I2C durante el init: %s", esp_err_to_name(i2c_err));
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sensor listo (timing budget = %lu us)", (unsigned long)timing_budget_us);
    return ESP_OK;
}


/* ====================================================================
 * API pública
 * ==================================================================== */

esp_err_t laser_set_budget_us(uint32_t us)
{
    if (dev == NULL)         return ESP_ERR_INVALID_STATE;   // primero laser_init()
    if (us < MIN_BUDGET_US)  return ESP_ERR_INVALID_ARG;

    i2c_err = ESP_OK;
    bool ok = set_timing_budget(us);
    if (i2c_err != ESP_OK) return i2c_err;
    return ok ? ESP_OK : ESP_ERR_INVALID_ARG;
}

//laser_medir: mide una vez (modo single) y dice si la medición sirve
laser_estado_t laser_medir(uint16_t *mm)
{
    i2c_err = ESP_OK;
    *mm = 0;

    // 1) Preparar el sensor con la stop variable que leímos en el init
    write_reg(0x80, 0x01);
    write_reg(0xFF, 0x01);
    write_reg(0x00, 0x00);
    write_reg(0x91, stop_variable);
    write_reg(0x00, 0x01);
    write_reg(0xFF, 0x00);
    write_reg(0x80, 0x00);

    // 2) "Medí": escribir 0x01 en el registro 0x00
    write_reg(SYSRANGE_START, 0x01);
    if (i2c_err != ESP_OK) return LASER_ERROR_I2C;

    int64_t t0 = ahora_ms();
    while (read_reg(SYSRANGE_START) & 0x01) {               // esperar a que arranque
        if (i2c_err != ESP_OK)                  return LASER_ERROR_I2C;
        if (ahora_ms() - t0 > POLL_TIMEOUT_MS)  return LASER_TIMEOUT;
    }

    // 3) Esperar a que el registro 0x13 diga "medición lista"
    while ((read_reg(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
        if (i2c_err != ESP_OK)                  return LASER_ERROR_I2C;
        if (ahora_ms() - t0 > POLL_TIMEOUT_MS)  return LASER_TIMEOUT;
        vTaskDelay(1);   // ceder la CPU un tick (10 ms con FREERTOS_HZ=100) en vez de preguntar sin parar
    }

    // 4) Leer el bloque de resultados de una (12 bytes desde 0x14):
    //    r[0]      = range status
    //    r[6..7]   = tasa de señal (MCPS, punto fijo 9.7)
    //    r[10..11] = distancia en mm
    uint8_t r[12] = { 0 };
    read_multi(RESULT_RANGE_STATUS, r, sizeof(r));

    // 5) "Limpiar": escribir 0x01 en 0x0B para la próxima medición
    write_reg(SYSTEM_INTERRUPT_CLEAR, 0x01);

    // Si la lectura falló, r[] no sirve: NO devolvemos 0 mm como si fuera válido
    if (i2c_err != ESP_OK) return LASER_ERROR_I2C;

    uint8_t  estado = (r[0] & 0x78) >> 3;
    ultima_senal    = ((uint16_t)r[6] << 8) | r[7];
    *mm             = ((uint16_t)r[10] << 8) | r[11];

    // 6) ¿Es válida?
    if (*mm >= FUERA_DE_RANGO_MM)   return LASER_SIN_OBJETO;   // no vio nada
    if (estado != RANGE_STATUS_OK)  return LASER_INVALIDA;     // el sensor no confía en ella
    return LASER_OK;
}

float laser_ultima_senal_mcps(void)
{
    return ultima_senal / 128.0f;   // 9.7 -> dividir por 2^7
}

const char *laser_estado_str(laser_estado_t e)
{
    switch (e) {
    case LASER_OK:         return "OK";
    case LASER_SIN_OBJETO: return "SIN_OBJETO";
    case LASER_INVALIDA:   return "INVALIDA";
    case LASER_ERROR_I2C:  return "ERROR_I2C";
    case LASER_TIMEOUT:    return "TIMEOUT";
    }
    return "?";
}
