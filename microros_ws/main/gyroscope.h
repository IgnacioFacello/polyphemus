#pragma once

typedef struct
{
    // Velocidad angular recibida [deg/s]
    float gx_dps;
    float gy_dps;
    float gz_dps;

    // Velocidad angular convertida [rad/s]
    float gx_rad_s;
    float gy_rad_s;
    float gz_rad_s;

    // Giro producido durante una muestra [rad]
    float delta_x_rad;
    float delta_y_rad;
    float delta_z_rad;

} gyroscope_output_t;


/**
 * Procesa las mediciones del giroscopio.
 *
 * gx_dps, gy_dps, gz_dps : velocidades angulares [deg/s]
 * dt_s                   : tiempo entre muestras [s]
 * out                    : estructura con los resultados
 */
void gyroscope_process(
    float gx_dps,
    float gy_dps,
    float gz_dps,
    float dt_s,
    gyroscope_output_t *out
);