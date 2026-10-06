#pragma once

typedef struct
{
    float magnitude_g;

    float nx;
    float ny;
    float nz;

    float roll_rad;
    float pitch_rad;

    float roll_deg;
    float pitch_deg;

} accel_output_t;


/**
 * ax_g Aceleración en X [g]
 * ay_g Aceleración en Y [g]
 * az_g Aceleración en Z [g]
 * out  Estructura donde se guardan los resultados
 */
 
void accel_process(
    float ax_g,
    float ay_g,
    float az_g,
    accel_output_t *out
);