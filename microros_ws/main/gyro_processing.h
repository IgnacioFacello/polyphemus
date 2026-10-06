#pragma once


typedef struct
{
    float roll_deg;
    float pitch_deg;
    float yaw_deg;

} gyro_output_t;


/**
 * Inicializa los ángulos estimados en cero.
 */
void gyro_reset(
    gyro_output_t *out
);


/**
 *  Integra las velocidades angulares del giroscopio.
 *
 *  gx_dps Velocidad angular X [deg/s]
 * gy_dps Velocidad angular Y [deg/s]
 * gz_dps Velocidad angular Z [deg/s]
 * dt_s   Tiempo entre muestras [s]
 *  out    Resultado de orientación
 */
void gyro_process(
    float gx_dps,
    float gy_dps,
    float gz_dps,
    float dt_s,
    gyro_output_t *out
);