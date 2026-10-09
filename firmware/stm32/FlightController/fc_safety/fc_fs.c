#include "fc_fs.h"

#include "fc_context.h"   // s, FC_FS_*
#include "fc_cfg.h"
#include "fc_evt.h"
#include "fc_util.h"
#include "fc_air.h"
#include "timebase.h"
#include "motors.h"

#include <math.h>

static inline void fs_cut_motors(void)
{
  motors_set_armed(0);
}

static inline void fs_invalidate_baro(void)
{
  s.baro.valid = 0;
  s.baro.ts_ticks = 0;
  s.baro.alt_rel_m = 0.0f;
  s.baro.vz_mps    = 0.0f;
}

void fc_reset_controller_states(void)
{
    s.ctrl.thr_smooth = 0.0f;
    s.ctrl.yaw_i_term = 0.0f;

    s.idle_ramp_t = 0.0f;
    s.was_armed   = 0;

    s.ctrl.roll_d_inited  = 0;
    s.ctrl.pitch_d_inited = 0;

    // ===== alt-hold reset =====
    s.ctrl.alt.active    = 0u;
    s.ctrl.alt.was_sw    = 0u;
    s.ctrl.alt.z_sp_m    = 0.0f;
    s.ctrl.alt.thr_mid   = 0.0f;
    s.ctrl.alt.thr_hover = 0.0f;
    s.ctrl.alt.i_term    = 0.0f;
    s.ctrl.alt.thr_out   = 0.0f;
	  s.ctrl.alt.enter_t  = 0.0f;   //ENTER保护期清零

    fc_air_reset();
}

void fc_clear_failsafe_latch(void)
{
  s.fs.latched         = FC_FS_NONE;
  s.fs.tilt_over_ticks = 0;

  s.arm.armed = 0;

  fc_reset_controller_states();
	fs_invalidate_baro();
  fs_cut_motors();
}

void fc_emergency_stop(uint32_t reason)
{
  s.fs.latched |= reason;

  s.arm.armed = 0;
  s.fs.tilt_over_ticks = 0;

  fc_reset_controller_states();
	fs_invalidate_baro();
  fs_cut_motors();
}

void fc_cut_disarmed(void) 
{
  s.arm.armed = 0;

  s.ctrl.thr_smooth = 0.0f;
  s.ctrl.yaw_i_term = 0.0f;

  /* ---- alt-hold reset ---- */
  s.ctrl.alt.active  = 0u;
  s.ctrl.alt.enter_t = 0.0f;
  s.ctrl.alt.i_term  = 0.0f;
  s.ctrl.alt.thr_out = 0.0f;
  s.ctrl.alt.z_sp_m    = 0.0f;
  s.ctrl.alt.thr_mid   = 0.0f;
  s.ctrl.alt.thr_hover = 0.0f;
  s.ctrl.alt.was_sw = s.rc.alt_sw;   // 推荐：要求重新拨一下开关

  s.idle_ramp_t = 0.0f;
  s.was_armed   = 0;

  s.ctrl.roll_d_inited   = 0;
  s.ctrl.pitch_d_inited  = 0;
  s.ctrl.roll_rate_prev  = 0.0f;
  s.ctrl.pitch_rate_prev = 0.0f;
  s.ctrl.roll_d_lpf      = 0.0f;
  s.ctrl.pitch_d_lpf     = 0.0f;

  s.fs.tilt_over_ticks = 0;

  fc_air_reset();
  fs_cut_motors();
}

fc_step_ret_t fc_fs_tilt_update(void)
{
  // 相当于你原来的 tilt_update()

  float thr_raw    = s.rc.thr;
  float thr_smooth = s.ctrl.thr_smooth;
  float thr_gate   = (thr_raw > thr_smooth) ? thr_raw : thr_smooth;

  float r = s.att.roll_deg_use;
  float p = s.att.pitch_deg_use;

  if (s.fs.tilt_time_ticks == 0) {
    s.fs.tilt_time_ticks = sec_to_ticks(TILT_TIME_SEC);
  }

  // 不允许计时的情况：直接清计时
  if (!s.arm.armed || (thr_gate < TILT_THR_MIN) || ((s.fs.latched & FC_FS_TILT) != 0u)) {
    s.fs.tilt_over_ticks = 0;
    return FC_STEP_CONTINUE;
  }

  // 超角判定
  uint8_t over = (fabsf(r) > TILT_CUTOFF_DEG) || (fabsf(p) > TILT_CUTOFF_DEG);

  if (!over) {
    s.fs.tilt_over_ticks = 0;
    return FC_STEP_CONTINUE;
  }

  // over：开始计时 or 达到持续时间 -> 锁存
  if (s.fs.tilt_over_ticks == 0) {
    s.fs.tilt_over_ticks = now_ticks();
    return FC_STEP_CONTINUE;
  }

  uint32_t t_now = now_ticks();
  if ((uint32_t)(t_now - s.fs.tilt_over_ticks) >= s.fs.tilt_time_ticks) {
    fc_evt_push(FC_EVT_TILT_LATCH, q10(r), q10(p), q100(thr_gate), 0);
    s.arm.pending_ahrs_reset = 1;
    fc_emergency_stop(FC_FS_TILT);
    return FC_STEP_STOP;
  }

  return FC_STEP_CONTINUE;
}
