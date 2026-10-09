#include "fc_yaw.h"

#include "stm32f4xx_hal.h"

#include "fc_context.h"
#include "fc_cfg.h"
#include "fc_evt.h"
#include "fc_util.h"

#include <math.h>

static inline float yaw_gain_compute(float thr_used, uint8_t air_on)
{
    if (air_on) {
        return 1.0f;
    }

    if (thr_used <= THR_LOW_GATE) {
        return 0.0f;
    }

    float denom = YAW_BLEND_THR - THR_LOW_GATE;
    if (denom < 1e-6f) {
        denom = 1e-6f;
    }

    float g = (thr_used - THR_LOW_GATE) / denom;

    if (g > 1.0f) {
        g = 1.0f;
    }

    if (g < 0.0f) {
        g = 0.0f;
    }

    return g;
}

void fc_yaw_update(float yaw_err,
                   float rc_yaw,
                   float thr_base,
                   float idle_now,
                   float dt,
                   uint8_t skip_id,
                   fc_yaw_out_t *out)
{
    const uint8_t air_eff = (uint8_t)(s.air.on || s.air.ever_on);

    float thr_used = thr_base;
    float yaw_gain = yaw_gain_compute(thr_used, air_eff);

    float yaw_err_norm = yaw_err / ACRO_MAX_RATE_YAW_DPS;
    float yaw_p = ACRO_KP_YAW * yaw_err_norm;

    float yaw_i_thr = air_eff ? idle_now : YAW_I_ENABLE_THR;

    if (!s.arm.armed || (thr_used < yaw_i_thr)) {
        s.ctrl.yaw_i_term = 0.0f;
    } else {
        if (fabsf(rc_yaw) < YAW_CENTER_DB) {
            float a = dt / (YAW_I_DECAY_TAU_SEC + dt);
            s.ctrl.yaw_i_term += (-s.ctrl.yaw_i_term) * a;
        }

        if (!skip_id) {
            float yaw_u = yaw_p + s.ctrl.yaw_i_term;

            int pushing_same_dir =
                (yaw_u >  1.0f && yaw_err_norm > 0.0f) ||
                (yaw_u < -1.0f && yaw_err_norm < 0.0f);

            if (!pushing_same_dir) {
                s.ctrl.yaw_i_term += ACRO_KI_YAW *
                                     yaw_err_norm *
                                     (dt / FC_DT_NOM_SEC);
            }
        }

        if (s.ctrl.yaw_i_term > ACRO_YAW_I_MAX) {
            s.ctrl.yaw_i_term = ACRO_YAW_I_MAX;
        }

        if (s.ctrl.yaw_i_term < -ACRO_YAW_I_MAX) {
            s.ctrl.yaw_i_term = -ACRO_YAW_I_MAX;
        }
    }

    out->yaw_out  = (yaw_p + s.ctrl.yaw_i_term) * yaw_gain;
    out->yaw_gain = yaw_gain;
    out->thr_used = thr_used;
    out->air_eff  = air_eff;
}

void fc_yaw_debug_poll(const fc_yaw_out_t *yaw_out, float rc_yaw)
{
    static uint32_t s_yaw_dbg_last_ms = 0;

    uint32_t now_ms = HAL_GetTick();

    if ((uint32_t)(now_ms - s_yaw_dbg_last_ms) >= YAW_DBG_EVERY_MS) {
        s_yaw_dbg_last_ms = now_ms;

        fc_evt_push(FC_EVT_YAW_DBG,
                    q1000(yaw_out->thr_used),
                    q1000(yaw_out->yaw_gain),
                    q1000(s.ctrl.yaw_i_term),
                    q1000(rc_yaw));
    }
}
