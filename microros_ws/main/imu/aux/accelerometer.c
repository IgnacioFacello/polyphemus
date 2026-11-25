#include "accelerometer.h"

#include <math.h>
#include <stddef.h>


#define RAD_TO_DEG 57.29577951308232f
#define ACCEL_EPSILON 0.0001f


void accelerometer_process(
    accelerometer_output_t *out)
{
    if (out == NULL)
    {
        return;
    }

    // Módulo del vector aceleración
    out->magnitude_g =
        sqrtf(
            out->ax_g * out->ax_g +
            out->ay_g * out->ay_g +
            out->az_g * out->az_g
        );


    // Normalización del vector aceleración
    if (out->magnitude_g > ACCEL_EPSILON)
    {
        out->nx = out->ax_g / out->magnitude_g;
        out->ny = out->ay_g / out->magnitude_g;
        out->nz = out->az_g / out->magnitude_g;
    }
    else
    {
        out->nx = 0.0f;
        out->ny = 0.0f;
        out->nz = 0.0f;
    }


    // Roll
    out->roll_rad =
        atan2f(
            out->ay_g,
            out->az_g
        );


    // Pitch
    out->pitch_rad =
        atan2f(
            -out->ax_g,
            sqrtf(
                out->ay_g * out->ay_g +
                out->az_g * out->az_g
            )
        );


    // Radianes -> grados
    out->roll_deg =
        out->roll_rad * RAD_TO_DEG;

    out->pitch_deg =
        out->pitch_rad * RAD_TO_DEG;
}
