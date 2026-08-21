#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

/* ADC */
#include "esp_adc/adc_oneshot.h"

/* Calibración del ADC */
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

/* DAC */
#include "driver/dac_oneshot.h"


static const char *TAG = "ADC";


void app_main(void)
{
    /* =====================================================
       1. CONFIGURACIÓN DEL ADC
       ADC1 - Canal 6 - GPIO34
       ===================================================== */

    adc_oneshot_unit_handle_t adc1_handle;

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };

    ESP_ERROR_CHECK(
        adc_oneshot_new_unit(
            &init_config,
            &adc1_handle
        )
    );


    adc_oneshot_chan_cfg_t channel_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            adc1_handle,
            ADC_CHANNEL_6,
            &channel_config
        )
    );


    /* =====================================================
       2. CALIBRACIÓN DEL ADC
       ESP32 clásico -> Line Fitting
       ===================================================== */

    adc_cali_handle_t adc_cali_handle = NULL;

    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .default_vref = 0,
    };

    ESP_ERROR_CHECK(
        adc_cali_create_scheme_line_fitting(
            &cali_config,
            &adc_cali_handle
        )
    );

    ESP_LOGI(TAG, "ADC calibration initialized");


    /* =====================================================
       3. CONFIGURACIÓN DEL DAC
       DAC canal 0 -> GPIO25
       ===================================================== */

    dac_oneshot_handle_t dac_handle;

    dac_oneshot_config_t dac_config = {
        .chan_id = DAC_CHAN_0,
    };

    ESP_ERROR_CHECK(
        dac_oneshot_new_channel(
            &dac_config,
            &dac_handle
        )
    );


    /* =====================================================
       4. VALORES DE PRUEBA DEL DAC

       0   -> aproximadamente 0 %
       64  -> aproximadamente 25 %
       128 -> aproximadamente 50 %
       192 -> aproximadamente 75 %
       255 -> aproximadamente 100 %
       ===================================================== */

    uint8_t dac_values[] = {
        0,
        64,
        128,
        192,
        255
    };


    /* =====================================================
       5. BUCLE PRINCIPAL
       ===================================================== */

    while (1)
    {
        int raw_value = 0;
        int voltage_mv = 0;

        float voltage = 0.0f;
        float position = 0.0f;


        for (int i = 0; i < 5; i++)
        {
            /* ---------------------------------------------
               Generar tensión analógica con el DAC
               GPIO25
               --------------------------------------------- */

            ESP_ERROR_CHECK(
                dac_oneshot_output_voltage(
                    dac_handle,
                    dac_values[i]
                )
            );


            /* Esperar un poco para estabilizar la señal */

            vTaskDelay(pdMS_TO_TICKS(100));


            /* ---------------------------------------------
               Leer el ADC
               GPIO34
               --------------------------------------------- */

            ESP_ERROR_CHECK(
                adc_oneshot_read(
                    adc1_handle,
                    ADC_CHANNEL_6,
                    &raw_value
                )
            );


            /* ---------------------------------------------
               Convertir RAW -> milivoltios calibrados
               --------------------------------------------- */

            ESP_ERROR_CHECK(
                adc_cali_raw_to_voltage(
                    adc_cali_handle,
                    raw_value,
                    &voltage_mv
                )
            );


            /* ---------------------------------------------
               Convertir milivoltios -> voltios
               --------------------------------------------- */

            voltage = voltage_mv / 1000.0f;


            /* ---------------------------------------------
               Calcular posición equivalente
               0 V -> 0 %
               3.3 V -> 100 %
               --------------------------------------------- */

            position = (voltage / 3.3f) * 100.0f;


            /* ---------------------------------------------
               Mostrar resultados
               --------------------------------------------- */

            ESP_LOGI(
                TAG,
                "DAC: %d | RAW: %d | Calibrated: %d mV | Voltage: %.3f V | Position: %.1f %%",
                dac_values[i],
                raw_value,
                voltage_mv,
                voltage,
                position
            );


            /* Esperar 2 segundos antes del siguiente punto */

            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
}