#include "fc_control.h"

#include "fc_context.h"
#include "fc_cfg.h"
#include "fc_util.h"
#include "fc_evt.h"
#include "fc_mode.h"
#include "fc_angle.h"
#include "fc_mixer_out.h"
#include "fc_rate.h"
#include "fc_yaw.h"

#include <math.h>

fc_step_ret_t fc_ctrl_run(const float gyr_dps[3], float dt, uint8_t skip_id)
{
  float rc_throttle = s.rc.thr;
  float rc_roll     = s.rc.roll;
  float rc_pitch    = s.rc.pitch;
  float rc_yaw      = s.rc.yaw;
	
  float roll_deg_use  = s.att.roll_deg_use;
  float pitch_deg_use = s.att.pitch_deg_use;

  /* 3.3 姿态外环：stick → 期望角度 → 期望角速度 */
  float roll_rate_cmd, pitch_rate_cmd, yaw_rate_cmd;
	
	if (fc_mode_get() == FC_MODE_ANGLE) {
    fc_angle_outer_update(rc_roll,
                          rc_pitch,
                          rc_yaw,
                          roll_deg_use,
                          pitch_deg_use,
                          &roll_rate_cmd,
                          &pitch_rate_cmd,
                          &yaw_rate_cmd);
} else {
    /* ===== ACRO(rate): stick -> rate ===== */
    roll_rate_cmd  = rc_roll  * ACRO_MAX_RATE_ROLL_DPS;
    pitch_rate_cmd = rc_pitch * ACRO_MAX_RATE_PITCH_DPS;
    yaw_rate_cmd   = rc_yaw   * ACRO_MAX_RATE_YAW_DPS;

    s.dbg.roll_deg_cmd   = 0.0f;
    s.dbg.roll_deg_meas  = 0.0f;
    s.dbg.pitch_deg_cmd  = 0.0f;
    s.dbg.pitch_deg_meas = 0.0f;
	}

	fc_rate_out_t rate_out;

if (fc_rate_update_roll_pitch(gyr_dps,
                              roll_rate_cmd,
                              pitch_rate_cmd,
                              yaw_rate_cmd,
                              dt,
                              skip_id,
                              &rate_out) != FC_STEP_CONTINUE) {
    return FC_STEP_STOP;
}

float roll_out  = rate_out.roll_out;
float pitch_out = rate_out.pitch_out;
float yaw_err   = rate_out.yaw_err;
	
	  // ===== idle_now / thr_base：在 yaw 逻辑和 mixer 都要用，提前算一次 =====
  float idle_now = 0.0f;
#if FC_IDLE_ENABLE
  {
    float k = (FC_IDLE_RAMP_SEC > 1e-3f) ? (s.idle_ramp_t / FC_IDLE_RAMP_SEC) : 1.0f;
    if (k < 0.0f) k = 0.0f;
    if (k > 1.0f) k = 1.0f;
    idle_now = FC_IDLE_THR * k;
  }
#endif

  float thr_base = s.ctrl.thr_smooth;
  if (thr_base < idle_now) thr_base = idle_now;
	
	fc_yaw_out_t yaw_res;

fc_yaw_update(yaw_err,
              rc_yaw,
              thr_base,
              idle_now,
              dt,
              skip_id,
              &yaw_res);

float yaw_out = yaw_res.yaw_out;


  /* 3.6 限幅 */
  if (roll_out  >  1.0f) roll_out  =  1.0f;
  if (roll_out  < -1.0f) roll_out  = -1.0f;
  if (pitch_out >  1.0f) pitch_out =  1.0f;
  if (pitch_out < -1.0f) pitch_out = -1.0f;
  if (yaw_out   >  1.0f) yaw_out   =  1.0f;
  if (yaw_out   < -1.0f) yaw_out   = -1.0f;

  /* 3.7 mixer */
  fc_mixer_cmd_t u;

#if DIRECT_MIXER_TEST
  if (s.arm.armed) {
    u.throttle = 0.20f;
    u.roll     = rc_roll;
    u.pitch    = rc_pitch;
    u.yaw      = rc_yaw;
  } else {
    u.throttle = 0.0f;
    u.roll = u.pitch = u.yaw = 0.0f;
  }
#else
	if (s.arm.armed) {
    if (!s.air.ever_on && (rc_throttle <= THR_LOW_GATE)) {
      u.throttle = idle_now;          // 你如果要真“怠速”，idle_now 不能是 0
      u.roll = u.pitch = u.yaw = 0.0f;
    } else {// 空中（airmode on 或油门较高）：保留姿态修正
      u.throttle = thr_base;          // 建议 thr_base = max(thr_smooth, idle_now)
      u.roll     = roll_out;
      u.pitch    = pitch_out;
      u.yaw      = yaw_out;
    }
	} else {
    u.throttle = 0.0f;
    u.roll = u.pitch = u.yaw = 0.0f;
	}

#endif

  const float out_max = ACRO_MOTOR_MAX;
const float out_min = s.arm.armed ? idle_now : 0.0f;

fc_mixer_output(&u, out_min, out_max);
	
	fc_yaw_debug_poll(&yaw_res, rc_yaw);
	
	// === 硬检测：同一帧(同一个 dt_update)是否调用了两次 controller_run ===
	static uint32_t last_frame_seen = 0;
	if (s.dt.frame_id == last_frame_seen) {
  // same-frame double entry: push warning event, throttled to ~1Hz (1kHz loop => 1000 frames)
		static uint32_t last_warn_frame = 0;
		if ((uint32_t)(s.dt.frame_id - last_warn_frame) >= 1000u) {
      last_warn_frame = s.dt.frame_id;
      fc_evt_push(FC_EVT_WARN_CTRL2X,
                  (int16_t)(s.dt.frame_id & 0xFFFFu),
                  (int16_t)((s.dt.frame_id >> 16) & 0xFFFFu),
                  0, 0);
		}
	}
	last_frame_seen = s.dt.frame_id;
  return FC_STEP_CONTINUE;
}
