#include "gyroscope.h"

#include <stddef.h>


#define DEG_TO_RAD 0.01745329251994329577f


void gyroscope_process(
    float dt_s,
    gyroscope_output_t *out)
{
    if (out == NULL)
    {
        return;
    }

    // deg/s -> rad/s
    out->gx_rad_s = out->gx_dps * DEG_TO_RAD;
    out->gy_rad_s = out->gy_dps * DEG_TO_RAD;
    out->gz_rad_s = out->gz_dps * DEG_TO_RAD;


    // Pequeña rotación ocurrida entre muestras
    if (dt_s > 0.0f)
    {
        out->delta_x_rad = out->gx_rad_s * dt_s;
        out->delta_y_rad = out->gy_rad_s * dt_s;
        out->delta_z_rad = out->gz_rad_s * dt_s;
    }
    else
    {
        out->delta_x_rad = 0.0f;
        out->delta_y_rad = 0.0f;
        out->delta_z_rad = 0.0f;
    }
}
