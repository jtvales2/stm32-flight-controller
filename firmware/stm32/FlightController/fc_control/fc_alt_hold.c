#include "fc_alt_hold.h"

#include "fc_context.h"
#include "fc_cfg.h"
#include "fc_evt.h"
#include "fc_util.h"

#include <math.h>

static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline float thr_to_unit_bipolar(float thr, float mid)
{
    if (thr >= mid) {
        float den = 1.0f - mid;
        return (den > 1e-3f) ? (thr - mid) / den : 0.0f;
    } else {
        float den = mid;
        return (den > 1e-3f) ? (thr - mid) / den : 0.0f;
    }
}

#define FC_ALT_DBG_PERIOD_SEC  0.10f

static float    s_alt_dbg_acc = 0.0f;
static uint16_t s_alt_dbg_seq = 0u;

static void alt_dbg_push(float manual_thr)
{
    int16_t seq = (int16_t)(++s_alt_dbg_seq);

    fc_evt_push(FC_EVT_ALT_DBG,
                seq,
                q100(s.baro.alt_rel_m),
                q100(s.ctrl.alt.z_sp_m),
                q100(s.baro.vz_mps));

    fc_evt_push(FC_EVT_ALT_DBG2,
                seq,
                q1000(s.ctrl.alt.thr_out),
                q1000(manual_thr),
                q100(s.ctrl.alt.enter_t));
}

static uint8_t alt_entry_block_reason(float manual_thr)
{
    if (!s.rc.stable) {
        return FC_ALT_ENTER_BLOCK_RC_UNSTABLE;
    }

    if (fabsf(s.baro.vz_mps) > FC_ALT_ENTER_MAX_VZ_MPS) {
        return FC_ALT_ENTER_BLOCK_VZ;
    }

    if ((manual_thr < FC_ALT_ENTER_THR_MIN) ||
        (manual_thr > FC_ALT_ENTER_THR_MAX)) {
        return FC_ALT_ENTER_BLOCK_THR;
    }

    if (s.baro.alt_rel_m < FC_ALT_ENTER_MIN_Z) {
        return FC_ALT_ENTER_BLOCK_LOW_ALT;
    }

    return 0u;
}
uint8_t fc_alt_hold_should_run(void)
{
    return (uint8_t)(s.arm.armed &&
                     s.baro.valid &&
                     s.air.ever_on &&
                     s.rc.alt_sw);
}

void fc_alt_hold_update(float manual_thr,
                        float dt,
                        uint8_t skip_id,
                        fc_alt_hold_out_t *out)
{
    float thr_target;

    uint8_t alt_sw;
    uint8_t gate_ok;
    uint8_t rising;
    uint8_t enter_block;

    float u;
    float vz_cmd;
    float thr_max;
    float thr_enter;

    float ez;
    float evz;
    float i_new;
    float thr_unsat;
    float thr_sat;
    int   pushing_more;

    float z_meas;
    float vz_hat;
    float vz_corr;
    float vz_sp;

    float cr;
    float cp;
    float tilt_cos;
    float thr_ff;

    const float deg2rad = 0.01745329252f;

    const float gnd_z_m   = 0.20f;
    const float gnd_vz_ms = 0.30f;

    if (out == 0) {
        return;
    }

    thr_target = manual_thr;

    alt_sw = (uint8_t)(s.rc.alt_sw);

    gate_ok = (uint8_t)(
        s.arm.armed &&
        s.baro.valid &&
        s.air.ever_on
    );

    /*
     * 1) ALT switch OFF -> always exit & reset
     */
    if (!alt_sw) {
        if (s.ctrl.alt.active) {
            fc_evt_push(FC_EVT_ALT_EXIT,
                        FC_ALT_EXIT_SWITCH_OFF,
                        q100(s.baro.alt_rel_m),
                        q100(s.baro.vz_mps),
                        0);
        }

        s.ctrl.alt.active  = 0u;
        s.ctrl.alt.enter_t = 0.0f;
        s.ctrl.alt.i_term  = 0.0f;
        s.ctrl.alt.thr_out = 0.0f;

        /* prev switch state = OFF */
        s.ctrl.alt.was_sw  = 0u;

        thr_target = manual_thr;
    }
    /*
     * 2) ALT switch ON but gate not OK
     *    -> exit & lock, require OFF->ON to re-enter
     */
    else if (!gate_ok) {
        if (s.ctrl.alt.active) {
            int16_t flags = 0;
            if (s.arm.armed)   flags |= 1;
            if (s.baro.valid)  flags |= 2;
            if (s.air.ever_on) flags |= 4;

            fc_evt_push(FC_EVT_ALT_EXIT,
                        FC_ALT_EXIT_GATE_BAD,
                        q100(s.baro.alt_rel_m),
                        q100(s.baro.vz_mps),
                        flags);
        }

        s.ctrl.alt.active  = 0u;
        s.ctrl.alt.enter_t = 0.0f;
        s.ctrl.alt.i_term  = 0.0f;
        s.ctrl.alt.thr_out = 0.0f;

        /* lock: since switch is ON, set prev=ON so no rising until toggle */
        s.ctrl.alt.was_sw  = 1u;

        thr_target = manual_thr;
    }
    /*
     * 3) ALT switch ON & gate OK -> may enter / run alt-hold
     */
    else {
        rising = (uint8_t)(!s.ctrl.alt.was_sw);

        /*
         * Rising edge capture.
         * Mature alt-hold must be entered deliberately while the craft is steady.
         * Any blocked attempt is locked until switch OFF->ON.
         */
        if (rising) {
            enter_block = alt_entry_block_reason(manual_thr);
            if (enter_block != 0u) {
                fc_evt_push(FC_EVT_ALT_ENTER_BLOCK,
                            enter_block,
                            q100(s.baro.alt_rel_m),
                            q100(s.baro.vz_mps),
                            q1000(manual_thr));

                s.ctrl.alt.active  = 0u;
                s.ctrl.alt.enter_t = 0.0f;
                s.ctrl.alt.i_term  = 0.0f;
                s.ctrl.alt.thr_out = 0.0f;

                /* lock: require OFF->ON after a rejected enter attempt */
                s.ctrl.alt.was_sw = 1u;
                thr_target = manual_thr;
            } else {
                s.ctrl.alt.active    = 1u;
                s.ctrl.alt.z_sp_m    = s.baro.alt_rel_m;
                s.ctrl.alt.thr_hover = s.ctrl.thr_smooth;
                s.ctrl.alt.thr_mid   = manual_thr;
                s.ctrl.alt.i_term    = 0.0f;
                s.ctrl.alt.enter_t   = FC_ALT_ENTER_SEC;

                fc_evt_push(FC_EVT_ALT_ENTER,
                            q1000(s.ctrl.alt.thr_mid),
                            q1000(s.ctrl.alt.thr_hover),
                            q100(s.baro.alt_rel_m),
                            q100(s.baro.vz_mps));

                s.ctrl.alt.was_sw = 1u;
            }
        } else {
            s.ctrl.alt.was_sw = 1u;
        }

        /*
         * If we still haven't entered, keep manual throttle.
         */
        if (!s.ctrl.alt.active) {
            thr_target = manual_thr;
        } else {
            /*
             * Near-ground safety / pogo fix.
             * Exit alt-hold and lock re-entry until toggle OFF->ON.
             */
            if ((s.baro.alt_rel_m < gnd_z_m) &&
                (fabsf(s.baro.vz_mps) < gnd_vz_ms)) {

                fc_evt_push(FC_EVT_ALT_EXIT,
                            FC_ALT_EXIT_POGO,
                            q100(s.baro.alt_rel_m),
                            q100(s.baro.vz_mps),
                            q1000(manual_thr));

                s.ctrl.alt.active  = 0u;
                s.ctrl.alt.enter_t = 0.0f;
                s.ctrl.alt.i_term  = 0.0f;
                s.ctrl.alt.thr_out = 0.0f;

                s.ctrl.alt.was_sw  = 1u;

                thr_target = manual_thr;
            } else {
                /*
                 * ENTER timer countdown
                 */
                if (s.ctrl.alt.enter_t > 0.0f) {
                    s.ctrl.alt.enter_t -= dt;
                    if (s.ctrl.alt.enter_t < 0.0f) {
                        s.ctrl.alt.enter_t = 0.0f;
                    }
                }

                /*
                 * Stick -> vertical speed command.
                 * Captured thr_mid is treated as zero climb/descent command.
                 */
                u = thr_to_unit_bipolar(manual_thr, s.ctrl.alt.thr_mid);
                if (fabsf(u) < FC_ALT_VZ_DB) {
                    u = 0.0f;
                }

                vz_cmd = u * FC_ALT_VZ_MAX;

                thr_max = ACRO_THR_MAX;
                if (thr_max > ACRO_MOTOR_MAX) {
                    thr_max = ACRO_MOTOR_MAX;
                }

                /*
                 * ENTER phase:
                 * lock current height, no I, only vertical-speed damping.
                 */
                if (s.ctrl.alt.enter_t > 0.0f) {
                    s.ctrl.alt.z_sp_m = s.baro.alt_rel_m;
                    s.ctrl.alt.i_term = 0.0f;

                    cr = cosf(s.att.roll_deg_use  * deg2rad);
                    cp = cosf(s.att.pitch_deg_use * deg2rad);
                    tilt_cos = cr * cp;
                    tilt_cos = clampf(tilt_cos, FC_ALT_TILT_COS_MIN, 1.0f);

                    thr_ff = s.ctrl.alt.thr_hover / tilt_cos;

                    vz_cmd = 0.0f;
                    thr_enter = thr_ff + FC_ALT_KP_VZ * (vz_cmd - s.baro.vz_mps);
                    thr_enter = clampf(thr_enter,
                                       thr_ff - FC_ALT_THR_CORR_MAX,
                                       thr_ff + FC_ALT_THR_CORR_MAX);

                    thr_target = clampf(thr_enter, 0.0f, thr_max);
                    s.ctrl.alt.thr_out = thr_target;
                } else {
                    /*
                     * After ENTER:
                     * stick moves target height, otherwise target height stays.
                     */
                    s.ctrl.alt.z_sp_m += vz_cmd * dt;
                    s.ctrl.alt.z_sp_m = clampf(s.ctrl.alt.z_sp_m,
                                               FC_ALT_Z_MIN,
                                               FC_ALT_Z_MAX);

                    /*
                     * Cascaded loops: z -> vz_sp -> throttle
                     */
                    z_meas = s.baro.alt_rel_m;
                    vz_hat = s.baro.vz_mps;

                    ez = s.ctrl.alt.z_sp_m - z_meas;
                    ez = clampf(ez, -1.5f, 1.5f);

                    vz_corr = FC_ALT_KZ_TO_VZ * ez;
                    vz_corr = clampf(vz_corr,
                                     -FC_ALT_VZ_CORR_MAX,
                                     FC_ALT_VZ_CORR_MAX);

                    vz_sp = vz_cmd + vz_corr;
                    vz_sp = clampf(vz_sp,
                                   -FC_ALT_VZ_MAX,
                                   FC_ALT_VZ_MAX);

                    evz = vz_sp - vz_hat;
                    evz = clampf(evz, -2.0f, 2.0f);

                    /*
                     * Integrate on vz loop.
                     * FC_ALT_KI_VZ can stay 0 for now.
                     */
                    i_new = s.ctrl.alt.i_term;

                    if (!skip_id) {
                        i_new += FC_ALT_KI_VZ * evz * dt;
                        i_new = clampf(i_new, -FC_ALT_I_MAX, FC_ALT_I_MAX);
                    }

                    /*
                     * Tilt compensation
                     */
                    cr = cosf(s.att.roll_deg_use  * deg2rad);
                    cp = cosf(s.att.pitch_deg_use * deg2rad);
                    tilt_cos = cr * cp;
                    tilt_cos = clampf(tilt_cos, FC_ALT_TILT_COS_MIN, 1.0f);

                    thr_ff = s.ctrl.alt.thr_hover / tilt_cos;

                    thr_unsat = thr_ff + (FC_ALT_KP_VZ * evz) + i_new;
                    thr_unsat = clampf(thr_unsat,
                                       thr_ff - FC_ALT_THR_CORR_MAX,
                                       thr_ff + FC_ALT_THR_CORR_MAX);
                    thr_sat   = clampf(thr_unsat, 0.0f, thr_max);

                    /*
                     * Anti-windup:
                     * if saturated and error is pushing further into saturation,
                     * freeze integrator.
                     */
                    if (!skip_id) {
                        pushing_more =
                            (thr_unsat > thr_max && evz > 0.0f) ||
                            (thr_unsat < 0.0f    && evz < 0.0f);

                        if (!pushing_more) {
                            s.ctrl.alt.i_term = i_new;
                        }
                    }

                    thr_target = thr_sat;
                    s.ctrl.alt.thr_out = thr_target;
                }
            }
        }
    }

    if (s.ctrl.alt.active) {
        s_alt_dbg_acc += dt;
        if (s_alt_dbg_acc >= FC_ALT_DBG_PERIOD_SEC) {
            s_alt_dbg_acc = 0.0f;
            alt_dbg_push(manual_thr);
        }
    } else {
        s_alt_dbg_acc = 0.0f;
    }

    out->thr_target = thr_target;
    out->active     = s.ctrl.alt.active;
}

