#pragma once

#include "accelerometer.h"
#include "gyroscope.h"
#include "quaternion.h"

#define COMPLEMENTARY_FILTER_DEFAULT_ALPHA 0.98f


/**
 * Actualiza la orientación fusionando
 * acelerómetro + giroscopio.
 *
 * q       : orientación actual y resultado final
 * accel   : datos procesados del acelerómetro
 * gyro    : datos procesados del giroscopio
 * dt_s    : tiempo entre muestras [s]
 * alpha   : peso del giroscopio (0.0 - 1.0)
 */
void complementary_filter_update(
    quaternion_t *q,
    const accelerometer_output_t *accel,
    const gyroscope_output_t *gyro,
    float dt_s,
    float alpha
);