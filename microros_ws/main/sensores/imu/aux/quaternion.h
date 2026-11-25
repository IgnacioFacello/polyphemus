#pragma once


typedef struct
{
    float w;
    float x;
    float y;
    float z;

} quaternion_t;


/**
 * Inicializa el cuaternión identidad.
 *
 * q = [1, 0, 0, 0]
 */
void quaternion_identity(
    quaternion_t *q
);


/**
 * Normaliza el cuaternión para mantener norma = 1.
 */
void quaternion_normalize(
    quaternion_t *q
);


/**
 * Multiplica dos cuaterniones.
 *
 * result = q1 ⊗ q2
 */
quaternion_t quaternion_multiply(
    quaternion_t q1,
    quaternion_t q2
);


/**
 * Calcula el conjugado.
 *
 * q* = [w, -x, -y, -z]
 */
quaternion_t quaternion_conjugate(
    quaternion_t q
);


/**
 * Producto punto entre dos cuaterniones.
 *
 * Se usará después en el filtro para manejar
 * que q y -q representan la misma orientación.
 */
float quaternion_dot(
    quaternion_t q1,
    quaternion_t q2
);


/**
 * Actualiza la orientación utilizando
 * la velocidad angular del giroscopio.
 *
 * gx_rad_s, gy_rad_s, gz_rad_s : [rad/s]
 * dt_s                         : [s]
 */
void quaternion_update_gyro(
    quaternion_t *q,
    float gx_rad_s,
    float gy_rad_s,
    float gz_rad_s,
    float dt_s
);


/**
 * Genera un cuaternión a partir de
 * roll, pitch y yaw.
 *
 * Ángulos en radianes.
 * Convención Z-Y-X.
 */
quaternion_t quaternion_from_euler(
    float roll_rad,
    float pitch_rad,
    float yaw_rad
);


/**
 * Obtiene yaw a partir del cuaternión.
 *
 * Resultado en radianes.
 */
float quaternion_get_yaw(
    quaternion_t q
);