#include "fc_rate.h"

#include "fc_context.h"
#include "fc_cfg.h"

#include <math.h>

fc_step_ret_t fc_rate_update_roll_pitch(const float gyr_dps[3],
                                        float roll_rate_cmd,
                                        float pitch_rate_cmd,
                                        float yaw_rate_cmd,
                                        float dt,
                                        uint8_t skip_id,
                                        fc_rate_out_t *out)
{
    float roll_rate_meas  = gyr_dps[0];
    float pitch_rate_meas = gyr_dps[1];
    float yaw_rate_meas   = -gyr_dps[2];

    if (!isfinite((double)roll_rate_meas) ||
        !isfinite((double)pitch_rate_meas) ||
        !isfinite((double)yaw_rate_meas)) {
        fc_emergency_stop(FC_FS_IMU);
        return FC_STEP_STOP;
    }

    if (fabsf(roll_rate_meas)  > GYR_ABS_MAX_DPS ||
        fabsf(pitch_rate_meas) > GYR_ABS_MAX_DPS ||
        fabsf(yaw_rate_meas)   > GYR_ABS_MAX_DPS) {
        fc_emergency_stop(FC_FS_IMU);
        return FC_STEP_STOP;
    }

    s.dbg.roll_rate_cmd   = roll_rate_cmd;
    s.dbg.roll_rate_meas  = roll_rate_meas;
    s.dbg.pitch_rate_cmd  = pitch_rate_cmd;
    s.dbg.pitch_rate_meas = pitch_rate_meas;

    float roll_err  = roll_rate_cmd  - roll_rate_meas;
    float pitch_err = pitch_rate_cmd - pitch_rate_meas;
    float yaw_err   = yaw_rate_cmd   - yaw_rate_meas;

    /* roll PD */
    float roll_p = ACRO_KP_ROLL * (roll_err / ACRO_MAX_RATE_ROLL_DPS);
    float roll_d = 0.0f;

    if (!s.ctrl.roll_d_inited || skip_id) {
        s.ctrl.roll_rate_prev = roll_rate_meas;
        s.ctrl.roll_d_lpf     = 0.0f;
        s.ctrl.roll_d_inited  = 1;
        roll_d = 0.0f;
    } else {
        float deriv = (roll_rate_meas - s.ctrl.roll_rate_prev) / dt;
        s.ctrl.roll_rate_prev = roll_rate_meas;

        float alpha = dt / (D_LPF_TAU_SEC + dt);
        s.ctrl.roll_d_lpf += alpha * (deriv - s.ctrl.roll_d_lpf);

        roll_d = -ACRO_KD_ROLL *
                 FC_DT_NOM_SEC *
                 (s.ctrl.roll_d_lpf / ACRO_MAX_RATE_ROLL_DPS);
    }

    /* pitch PD */
    float pitch_p = ACRO_KP_PITCH * (pitch_err / ACRO_MAX_RATE_PITCH_DPS);
    float pitch_d = 0.0f;

    if (!s.ctrl.pitch_d_inited || skip_id) {
        s.ctrl.pitch_rate_prev = pitch_rate_meas;
        s.ctrl.pitch_d_lpf     = 0.0f;
        s.ctrl.pitch_d_inited  = 1;
        pitch_d = 0.0f;
    } else {
        float deriv = (pitch_rate_meas - s.ctrl.pitch_rate_prev) / dt;
        s.ctrl.pitch_rate_prev = pitch_rate_meas;

        float alpha = dt / (D_LPF_TAU_SEC + dt);
        s.ctrl.pitch_d_lpf += alpha * (deriv - s.ctrl.pitch_d_lpf);

        pitch_d = -ACRO_KD_PITCH *
                  FC_DT_NOM_SEC *
                  (s.ctrl.pitch_d_lpf / ACRO_MAX_RATE_PITCH_DPS);
    }

    out->roll_out      = roll_p + roll_d;
    out->pitch_out     = pitch_p + pitch_d;
    out->yaw_err       = yaw_err;
    out->yaw_rate_meas = yaw_rate_meas;

    return FC_STEP_CONTINUE;
}
