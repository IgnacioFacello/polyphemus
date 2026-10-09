#include "quaternion.h"

#include <math.h>
#include <stddef.h>


#define QUATERNION_EPSILON 0.000001f


void quaternion_identity(
    quaternion_t *q)
{
    if (q == NULL)
    {
        return;
    }

    q->w = 1.0f;
    q->x = 0.0f;
    q->y = 0.0f;
    q->z = 0.0f;
}


void quaternion_normalize(
    quaternion_t *q)
{
    if (q == NULL)
    {
        return;
    }

    float norm =
        sqrtf(
            q->w * q->w +
            q->x * q->x +
            q->y * q->y +
            q->z * q->z
        );

    if (norm > QUATERNION_EPSILON)
    {
        q->w /= norm;
        q->x /= norm;
        q->y /= norm;
        q->z /= norm;
    }
    else
    {
        quaternion_identity(q);
    }
}


quaternion_t quaternion_multiply(
    quaternion_t q1,
    quaternion_t q2)
{
    quaternion_t result;

    result.w =
        q1.w * q2.w -
        q1.x * q2.x -
        q1.y * q2.y -
        q1.z * q2.z;

    result.x =
        q1.w * q2.x +
        q1.x * q2.w +
        q1.y * q2.z -
        q1.z * q2.y;

    result.y =
        q1.w * q2.y -
        q1.x * q2.z +
        q1.y * q2.w +
        q1.z * q2.x;

    result.z =
        q1.w * q2.z +
        q1.x * q2.y -
        q1.y * q2.x +
        q1.z * q2.w;

    return result;
}


quaternion_t quaternion_conjugate(
    quaternion_t q)
{
    quaternion_t result;

    result.w =  q.w;
    result.x = -q.x;
    result.y = -q.y;
    result.z = -q.z;

    return result;
}


float quaternion_dot(
    quaternion_t q1,
    quaternion_t q2)
{
    return
        q1.w * q2.w +
        q1.x * q2.x +
        q1.y * q2.y +
        q1.z * q2.z;
}


void quaternion_update_gyro(
    quaternion_t *q,
    float gx_rad_s,
    float gy_rad_s,
    float gz_rad_s,
    float dt_s)
{
    if (q == NULL || dt_s <= 0.0f)
    {
        return;
    }


    // Cuaternión puro de velocidad angular
    quaternion_t omega =
    {
        .w = 0.0f,
        .x = gx_rad_s,
        .y = gy_rad_s,
        .z = gz_rad_s
    };


    // q ⊗ omega
    quaternion_t q_omega =
        quaternion_multiply(*q, omega);


    // Integración de Euler:
    // q_nuevo = q + 0.5 * (q ⊗ omega) * dt

    q->w += 0.5f * q_omega.w * dt_s;
    q->x += 0.5f * q_omega.x * dt_s;
    q->y += 0.5f * q_omega.y * dt_s;
    q->z += 0.5f * q_omega.z * dt_s;


    // Mantener cuaternión unitario
    quaternion_normalize(q);
}


quaternion_t quaternion_from_euler(
    float roll_rad,
    float pitch_rad,
    float yaw_rad)
{
    quaternion_t q_roll =
    {
        .w = cosf(roll_rad / 2.0f),
        .x = sinf(roll_rad / 2.0f),
        .y = 0.0f,
        .z = 0.0f
    };


    quaternion_t q_pitch =
    {
        .w = cosf(pitch_rad / 2.0f),
        .x = 0.0f,
        .y = sinf(pitch_rad / 2.0f),
        .z = 0.0f
    };


    quaternion_t q_yaw =
    {
        .w = cosf(yaw_rad / 2.0f),
        .x = 0.0f,
        .y = 0.0f,
        .z = sinf(yaw_rad / 2.0f)
    };


    // Convención Z-Y-X:
    //
    // q = q_yaw ⊗ q_pitch ⊗ q_roll

    quaternion_t temp =
        quaternion_multiply(
            q_pitch,
            q_roll
        );

    quaternion_t result =
        quaternion_multiply(
            q_yaw,
            temp
        );


    quaternion_normalize(&result);

    return result;
}


float quaternion_get_yaw(
    quaternion_t q)
{
    return atan2f(
        2.0f * (q.w * q.z + q.x * q.y),
        1.0f - 2.0f * (q.y * q.y + q.z * q.z)
    );
}