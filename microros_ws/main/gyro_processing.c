#include "gyro_processing.h"


void gyro_reset(
    gyro_output_t *out)
{
    out->roll_deg = 0.0f;
    out->pitch_deg = 0.0f;
    out->yaw_deg = 0.0f;
}


void gyro_process(
    float gx_dps,
    float gy_dps,
    float gz_dps,
    float dt_s,
    gyro_output_t *out)
{
    out->roll_deg +=
        gx_dps * dt_s;

    out->pitch_deg +=
        gy_dps * dt_s;

    out->yaw_deg +=
        gz_dps * dt_s;
}