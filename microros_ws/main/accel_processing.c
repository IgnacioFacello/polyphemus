#include "accel_processing.h"

#include <math.h>


#define RAD_TO_DEG (180.0f / 3.14159265358979323846f)


void accel_process(
    float ax_g,
    float ay_g,
    float az_g,
    accel_output_t *out)
{
    /*
     * 1. Módulo del vector aceleración
     */

    out->magnitude_g =
        sqrtf(
            ax_g * ax_g +
            ay_g * ay_g +
            az_g * az_g
        );


    /*
     * 2. Normalización del vector
     * nx = ax / |a|
     * ny = ay / |a|
     * nz = az / |a|
     */

    if (out->magnitude_g > 0.0001f)
    {
        out->nx =
            ax_g / out->magnitude_g;

        out->ny =
            ay_g / out->magnitude_g;

        out->nz =
            az_g / out->magnitude_g;
    }
    else
    {
        out->nx = 0.0f;
        out->ny = 0.0f;
        out->nz = 0.0f;
    }


    /*
     * 3. Cálculo de Roll
     */

    out->roll_rad =
        atan2f(
            ay_g,
            az_g
        );


    /*
     * 4. Cálculo de Pitch
     */

    out->pitch_rad =
        atan2f(
            -ax_g,
            sqrtf(
                ay_g * ay_g +
                az_g * az_g
            )
        );


    /*
     * 5. Conversión radianes -> grados
     */

    out->roll_deg =
        out->roll_rad * RAD_TO_DEG;

    out->pitch_deg =
        out->pitch_rad * RAD_TO_DEG;
}