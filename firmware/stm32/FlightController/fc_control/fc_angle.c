#include "fc_angle.h"

#include "fc_context.h"
#include "fc_cfg.h"

void fc_angle_outer_update(float rc_roll,
                           float rc_pitch,
                           float rc_yaw,
                           float roll_deg_meas,
                           float pitch_deg_meas,
                           float *roll_rate_cmd,
                           float *pitch_rate_cmd,
                           float *yaw_rate_cmd)
{
    float roll_deg_cmd;
    float pitch_deg_cmd;
    float roll_deg_err;
    float pitch_deg_err;

    roll_deg_cmd  = rc_roll  * ANGLE_MAX_ROLL_DEG;
    pitch_deg_cmd = rc_pitch * ANGLE_MAX_PITCH_DEG;

    roll_deg_err  = roll_deg_cmd  - roll_deg_meas;
    pitch_deg_err = pitch_deg_cmd - pitch_deg_meas;

    *roll_rate_cmd  = ANGLE_KP_ROLL  * roll_deg_err;
    *pitch_rate_cmd = ANGLE_KP_PITCH * pitch_deg_err;

    if (*roll_rate_cmd > ACRO_MAX_RATE_ROLL_DPS) {
        *roll_rate_cmd = ACRO_MAX_RATE_ROLL_DPS;
    }
    if (*roll_rate_cmd < -ACRO_MAX_RATE_ROLL_DPS) {
        *roll_rate_cmd = -ACRO_MAX_RATE_ROLL_DPS;
    }

    if (*pitch_rate_cmd > ACRO_MAX_RATE_PITCH_DPS) {
        *pitch_rate_cmd = ACRO_MAX_RATE_PITCH_DPS;
    }
    if (*pitch_rate_cmd < -ACRO_MAX_RATE_PITCH_DPS) {
        *pitch_rate_cmd = -ACRO_MAX_RATE_PITCH_DPS;
    }

    *yaw_rate_cmd = rc_yaw * ACRO_MAX_RATE_YAW_DPS;

    s.dbg.roll_deg_cmd   = roll_deg_cmd;
    s.dbg.roll_deg_meas  = roll_deg_meas;
    s.dbg.pitch_deg_cmd  = pitch_deg_cmd;
    s.dbg.pitch_deg_meas = pitch_deg_meas;
}
