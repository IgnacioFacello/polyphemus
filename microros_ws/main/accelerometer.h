#pragma once

typedef struct
{
    // Aceleración recibida [g]
    float ax_g;
    float ay_g;
    float az_g;

    // Módulo del vector aceleración [g]
    float magnitude_g;

    // Dirección normalizada de la aceleración
    float nx;
    float ny;
    float nz;

    // Inclinación
    float roll_rad;
    float pitch_rad;

    float roll_deg;
    float pitch_deg;

} accelerometer_output_t;


/**
 * Procesa las mediciones del acelerómetro.
 *
 * ax_g, ay_g, az_g : aceleraciones en [g]
 * out              : estructura con los resultados
 */
void accelerometer_process(
    float ax_g,
    float ay_g,
    float az_g,
    accelerometer_output_t *out
);

