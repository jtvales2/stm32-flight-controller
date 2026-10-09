#ifndef FC_ANGLE_H
#define FC_ANGLE_H

#ifdef __cplusplus
extern "C" {
#endif

void fc_angle_outer_update(float rc_roll,
                           float rc_pitch,
                           float rc_yaw,
                           float roll_deg_meas,
                           float pitch_deg_meas,
                           float *roll_rate_cmd,
                           float *pitch_rate_cmd,
                           float *yaw_rate_cmd);

#ifdef __cplusplus
}
#endif

#endif
