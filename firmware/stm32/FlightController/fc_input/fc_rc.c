#include "fc_rc.h"

#include "fc_context.h"   // s、fc_emergency_stop
#include "fc_cfg.h"         // THR_LOW_GATE / YAW_DEADBAND / SBUS_RAW_* / RC_STABLE_AFTER_LINK_SEC
#include "fc_evt.h"
#include "sbus.h"
#include "timebase.h"

#include <math.h>

static float apply_deadband_rescale(float x, float db)
{
  float ax = fabsf(x);
  if (ax <= db) return 0.0f;

  /* rescale so full stick still reaches 1.0 */
  if (x > 0.0f) return (x - db) / (1.0f - db);
  else          return (x + db) / (1.0f - db);
}

/* us -> normalized */
static float rc_us_to_0_1(uint16_t us)
{
  float x = ((float)us - 1000.0f) / 1000.0f;
  if (x < 0.0f) x = 0.0f;
  if (x > 1.0f) x = 1.0f;
  return x;
}

static float rc_us_to_m1_p1(uint16_t us)
{
  float x = ((float)us - 1500.0f) / 500.0f;
  if (x < -1.0f) x = -1.0f;
  if (x >  1.0f) x =  1.0f;
  return x;
}

static float get_rc_throttle_norm(void)
{
  if (!s.rc.valid) return 0.0f;
  uint16_t us = sbus_to_us(s.rc.fr.ch[2]); // CH3
  float x = rc_us_to_0_1(us);
  if (x <= THR_LOW_GATE) x = 0.0f;
  return x;
}

static float get_rc_roll_norm(void)
{
  float x;
  if (!s.rc.valid) return 0.0f;
  x = rc_us_to_m1_p1(sbus_to_us(s.rc.fr.ch[0])); // CH1
  x = apply_deadband_rescale(x, RP_DEADBAND);
  return x;
}

static float get_rc_pitch_norm(void)
{
  float x;
  if (!s.rc.valid) return 0.0f;
  x = rc_us_to_m1_p1(sbus_to_us(s.rc.fr.ch[1])); // CH2
  x = -x;
  x = apply_deadband_rescale(x, RP_DEADBAND);
  return x;
}

static float get_rc_yaw_norm(void)
{
  if (!s.rc.valid) return 0.0f;
  uint16_t us = sbus_to_us(s.rc.fr.ch[3]); // CH4
  float x = rc_us_to_m1_p1(us);
  if (fabsf(x) < YAW_DEADBAND) x = 0.0f;
  return x;
}

static int get_rc_arm_switch(void)
{
  if (!s.rc.valid) return 0;
  uint16_t us = sbus_to_us(s.rc.fr.ch[4]); // CH5
  return (us > 1500);
}

typedef struct {
  uint8_t mode_sw;  /* 0=ACRO, 1=ANGLE */
  uint8_t alt_sw;   /* 1=ALT_HOLD request */
} rc_mode3_t;

static uint16_t get_rc_channel_us(uint8_t ch, uint16_t fallback_us)
{
  if (!s.rc.valid || ch >= 16u) return fallback_us;
  return sbus_to_us(s.rc.fr.ch[ch]);
}

static rc_mode3_t get_rc_mode3_switch(void)
{
  rc_mode3_t out = {0u, 0u};
  uint16_t us = get_rc_channel_us((uint8_t)FC_CFG_MODE_CH, 1000u); /* CH6 */

  if (us < (uint16_t)FC_MODE_LOW_MAX_US) {
    out.mode_sw = 0u; /* low: ACRO/rate */
    out.alt_sw  = 0u;
  } else if (us >= (uint16_t)FC_MODE_HIGH_MIN_US) {
    out.mode_sw = 1u; /* high: ANGLE + ALT_HOLD */
    out.alt_sw  = 1u;
  } else {
    out.mode_sw = 1u; /* middle: ANGLE */
    out.alt_sw  = 0u;
  }

  return out;
}
static int get_rc_level_switch(void)
{
  uint16_t us = get_rc_channel_us((uint8_t)FC_CFG_LEVEL_CH, 1000u); /* CH7 */
  return (us > 1500u) ? 1 : 0;
}

static int get_rc_esc_cal_switch(void)
{
  uint16_t us = get_rc_channel_us((uint8_t)FC_CFG_ESC_CAL_CH, 1000u); /* CH8 */
  return (us > 1500u) ? 1 : 0;
}

static uint8_t sbus_ch_sane(const sbus_frame_t *fr, uint8_t ch)
{
  if (ch >= 16u) return 1u;
  uint16_t v = fr->ch[ch];
  return (uint8_t)((v >= SBUS_RAW_MIN_SANE) && (v <= SBUS_RAW_MAX_SANE));
}

static uint8_t sbus_frame_sane(const sbus_frame_t *fr)
{
  const uint8_t fixed[] = {0,1,2,3,4}; // roll,pitch,thr,yaw,arm
  for (unsigned i = 0; i < (sizeof(fixed)/sizeof(fixed[0])); i++) {
    if (!sbus_ch_sane(fr, fixed[i])) return 0;
  }

  if (!sbus_ch_sane(fr, (uint8_t)FC_CFG_MODE_CH))    return 0;
  if (!sbus_ch_sane(fr, (uint8_t)FC_CFG_LEVEL_CH))   return 0;
  if (!sbus_ch_sane(fr, (uint8_t)FC_CFG_ESC_CAL_CH)) return 0;

  return 1;
}

fc_step_ret_t fc_rc_update(void)
{
  if (s.rc.timeout_ticks == 0) {
    s.rc.timeout_ticks = sec_to_ticks(0.200f);   // 默认 200ms
  }

  uint32_t now = now_ticks();

  // 快照：用于 age_ms 计算（避免后面清零影响）
  uint32_t last_good_snap = s.rc.last_rx_ticks;

  uint8_t fs_now   = 0;
  uint8_t lost_now = 0;
  uint8_t sane_now = 0;
  uint8_t got_new  = 0;

  sbus_frame_t fr;
  if (sbus_read_latest(&fr)) {
    got_new  = 1;
    fs_now   = fr.failsafe   ? 1u : 0u;
    lost_now = fr.lost_frame ? 1u : 0u;
    sane_now = sbus_frame_sane(&fr) ? 1u : 0u;

    // 当前行为：只有 非failsafe + 非lost + sane 才算好帧
    if (!fs_now && !lost_now && sane_now) {
      s.rc.fr            = fr;
      s.rc.last_rx_ticks = now;
      last_good_snap     = now;
    }
  }

  // valid 判定
  uint8_t valid = 0;
  if (fs_now) {
    valid = 0;
  } else if (s.rc.last_rx_ticks != 0) {
    valid = ((uint32_t)(now - s.rc.last_rx_ticks) <= s.rc.timeout_ticks);
  } else {
    valid = 0;
  }
  s.rc.valid = valid;

  // 重连稳定窗口
  static uint8_t last_valid = 0;
  if (s.rc.valid && !last_valid) {
    s.rc.link_up_ticks = now;
  }
  if (!s.rc.valid) {
    s.rc.link_up_ticks = 0;
    s.rc.stable = 0;
  } else {
    s.rc.stable = (uint8_t)((uint32_t)(now - s.rc.link_up_ticks) >=
                            sec_to_ticks(RC_STABLE_AFTER_LINK_SEC));
  }
  last_valid = s.rc.valid;
	
	static uint8_t s_mode_inited = 0;
	static uint8_t s_mode_last   = 0xFF;  // 0xFF 表示未初始化
	static uint8_t last_stable   = 0;
	
  // invalid：只在 valid->invalid 的瞬间打一条 FAILSAFE 事件，然后按 armed 状态 cut/stop
  static uint8_t was_valid = 1;
  if (!s.rc.valid) {

    if (was_valid) {
      int16_t age_ms = 32767;
      if (last_good_snap != 0) {
        uint32_t age_ticks = (uint32_t)(now - last_good_snap);
        float age = ticks_to_sec(age_ticks);
        int32_t ms = (int32_t)(age * 1000.0f);
        if (ms < 0) ms = 0;
        if (ms > 32767) ms = 32767;
        age_ms = (int16_t)ms;
      }

      // reason: 1=SBUS_FAILSAFE_BIT, 2=SBUS_TIMEOUT
      int16_t reason = fs_now ? 1 : 2;

      fc_evt_push(FC_EVT_FAILSAFE,
                  reason,
                  (int16_t)fs_now,
                  (int16_t)(got_new ? lost_now : 0),
                  age_ms);
    }
    was_valid = 0;

    // 关键：清掉 last_rx_ticks，强制后续走“重连 + stable 窗口”
    s.rc.last_rx_ticks = 0;

    if (s.arm.armed) fc_emergency_stop(FC_FS_LINK);
    else             fc_cut_disarmed();     // 下面第 3) 步会让它跨文件可见
		
		s_mode_inited = 0;
		s_mode_last   = 0xFF;
		
    return FC_STEP_STOP;
  }

  was_valid = 1;

  // 归一化 + 写回
  float rc_throttle = get_rc_throttle_norm();
  float rc_roll     = get_rc_roll_norm();
  float rc_pitch    = get_rc_pitch_norm();
  float rc_yaw      = get_rc_yaw_norm();
  int   arm_sw      = get_rc_arm_switch();
  rc_mode3_t mode3  = get_rc_mode3_switch();
  int   level_sw    = get_rc_level_switch();
  int   esc_cal_sw  = get_rc_esc_cal_switch();
  int   thr_low     = (rc_throttle <= 0.0f);
	
	uint8_t mode_now = mode3.mode_sw;   // CH6 low=ACRO, mid/high=ANGLE

  // 第一次 RC 进入 stable：打 INIT
  if (!last_stable && s.rc.stable) {
    fc_evt_push(FC_EVT_MODE, (int16_t)mode_now, (int16_t)FC_MODE_REASON_INIT, 0, 0);
    s_mode_inited = 1;
    s_mode_last   = mode_now;
  }
  last_stable = s.rc.stable;

  // 后续切换：打 SWITCH
  if (s_mode_inited && (mode_now != s_mode_last)) {
    fc_evt_push(FC_EVT_MODE, (int16_t)mode_now, (int16_t)FC_MODE_REASON_SWITCH, 0, 0);
    s_mode_last = mode_now;
  }
	
  s.rc.thr      = rc_throttle;
  s.rc.roll     = rc_roll;
  s.rc.pitch    = rc_pitch;
  s.rc.yaw      = rc_yaw;
  s.rc.arm_sw   = (uint8_t)arm_sw;
  s.rc.mode_sw  = mode_now;
  s.rc.alt_sw   = mode3.alt_sw;
  s.rc.level_sw = (uint8_t)level_sw;
  s.rc.esc_cal_sw = (uint8_t)esc_cal_sw;
  s.rc.thr_low  = (uint8_t)thr_low;

  return FC_STEP_CONTINUE;
}
