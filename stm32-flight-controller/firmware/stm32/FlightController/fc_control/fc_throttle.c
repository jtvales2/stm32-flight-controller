#include "fc_throttle.h"

#include "fc_context.h"
#include "fc_cfg.h"
#include "fc_alt_hold.h"

#include <math.h>


void fc_ctrl_thr_update(float dt, uint8_t skip_id)
{
    float manual_thr;
    float thr_target;
    float thr_max2;
    float tau;
    float alpha;

    fc_alt_hold_out_t alt_out;

    manual_thr = s.rc.thr;
    thr_target = manual_thr;

    /*
     * Alt-hold update is called every frame.
     * It handles:
     * - switch OFF reset
     * - switch ON but gate not OK lock
     * - OFF->ON rising edge
     * - active alt-hold control
     *
     * This keeps was_sw / re-entry lock behavior correct.
     */
    alt_out.thr_target = manual_thr;
    alt_out.active     = 0u;

    fc_alt_hold_update(manual_thr, dt, skip_id, &alt_out);

    thr_target = alt_out.thr_target;

    /*
     * Final throttle limit and smoothing.
     * This remains in fc_throttle.c, not in fc_alt_hold.c.
     */
    thr_max2 = ACRO_THR_MAX;
    if (thr_max2 > ACRO_MOTOR_MAX) {
        thr_max2 = ACRO_MOTOR_MAX;
    }

    if (thr_target > thr_max2) {
        thr_target = thr_max2;
    }

    if (thr_target < 0.0f) {
        thr_target = 0.0f;
    }

    tau = (thr_target > s.ctrl.thr_smooth) ?
          THR_TAU_UP_SEC :
          THR_TAU_DOWN_SEC;

    alpha = dt / (tau + dt);

    if (skip_id) {
        alpha = 1.0f;
    }

    s.ctrl.thr_smooth += alpha * (thr_target - s.ctrl.thr_smooth);

    if (s.ctrl.thr_smooth < 0.0f) {
        s.ctrl.thr_smooth = 0.0f;
    }

    if (s.ctrl.thr_smooth > thr_max2) {
        s.ctrl.thr_smooth = thr_max2;
    }
}

void fc_ctrl_idle_ramp_update(float dt)
{
#if !FC_IDLE_ENABLE
    s.idle_ramp_t = 0.0f;
    s.was_armed   = 0;
    return;
#endif

    if (!s.arm.armed) {
        s.idle_ramp_t = 0.0f;
        s.was_armed   = 0;
        return;
    }

    /*
     * 刚解锁的第一帧：从 0 开始。
     * 这一帧先不累加，避免“解锁瞬间 idle 跳一下”。
     */
    if (!s.was_armed) {
        s.was_armed   = 1;
        s.idle_ramp_t = 0.0f;
        return;
    }

    if (s.idle_ramp_t < FC_IDLE_RAMP_SEC) {
        s.idle_ramp_t += dt;
        if (s.idle_ramp_t > FC_IDLE_RAMP_SEC) {
            s.idle_ramp_t = FC_IDLE_RAMP_SEC;
        }
    }
}
