#include "complementary_filter.h"

#include <stddef.h>


void complementary_filter_update(
    quaternion_t *q,
    const accelerometer_output_t *accel,
    const gyroscope_output_t *gyro,
    float dt_s,
    float alpha)
{
    if (q == NULL ||
        accel == NULL ||
        gyro == NULL ||
        dt_s <= 0.0f)
    {
        return;
    }


    // Limitar alpha entre 0 y 1
    if (alpha < 0.0f)
    {
        alpha = 0.0f;
    }
    else if (alpha > 1.0f)
    {
        alpha = 1.0f;
    }


    // 1. Predicción de orientación usando el giroscopio
    quaternion_t q_gyro = *q;

    quaternion_update_gyro(
        &q_gyro,
        gyro->gx_rad_s,
        gyro->gy_rad_s,
        gyro->gz_rad_s,
        dt_s
    );


    // 2. El acelerómetro no puede determinar yaw.
    // Se conserva el yaw estimado por el giroscopio.
    float yaw_rad =
        quaternion_get_yaw(q_gyro);


    // 3. Crear orientación basada en:
    // roll  -> acelerómetro
    // pitch -> acelerómetro
    // yaw   -> giroscopio
    quaternion_t q_acc =
        quaternion_from_euler(
            accel->roll_rad,
            accel->pitch_rad,
            yaw_rad
        );


    // 4. q y -q representan la misma orientación.
    // Los colocamos del mismo lado antes de mezclarlos.
    if (quaternion_dot(q_gyro, q_acc) < 0.0f)
    {
        q_acc.w = -q_acc.w;
        q_acc.x = -q_acc.x;
        q_acc.y = -q_acc.y;
        q_acc.z = -q_acc.z;
    }


    // 5. Filtro complementario
    q->w =
        alpha * q_gyro.w +
        (1.0f - alpha) * q_acc.w;

    q->x =
        alpha * q_gyro.x +
        (1.0f - alpha) * q_acc.x;

    q->y =
        alpha * q_gyro.y +
        (1.0f - alpha) * q_acc.y;

    q->z =
        alpha * q_gyro.z +
        (1.0f - alpha) * q_acc.z;


    // 6. Mantener el cuaternión unitario
    quaternion_normalize(q);
}