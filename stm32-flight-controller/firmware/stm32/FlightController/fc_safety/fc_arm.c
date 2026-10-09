#include "fc_arm.h"
#include <math.h>
#include "sync_pair.h"     // imu_get_gyro_dps()
#include "autocal.h"
#include "imu_bringup.h"   // imu_get_gyro_bias_dps(), imu_set_gyro_bias_dps()
#include "fc_evt.h"
#include "fc_context.h"   // s, failsafe bits, 内部 helper 声明
#include "fc_att.h"         // fc_is_level_ok()
#include "fc_cfg.h"
#include "fc_evt.h"
#include "fc_util.h"
#include "timebase.h"
#include "motors.h"
#include "fusion_mahony.h"
#include <stddef.h>   // for NULL

extern Mahony mah;

static uint32_t s_prearm_t0 = 0u;
static uint16_t s_prearm_n = 0u;
static float s_prearm_sum[3] = {0.0f, 0.0f, 0.0f};

static void fc_prearm_cal_reset(void)
{
  s_prearm_t0 = 0u;
  s_prearm_n = 0u;
  s_prearm_sum[0] = 0.0f;
  s_prearm_sum[1] = 0.0f;
  s_prearm_sum[2] = 0.0f;
}

static void fc_q_from_level_offset(float q[4])
{
  const float deg2rad = 0.01745329252f;
  float roll = s.level.has_off ? (s.level.off_roll_deg * deg2rad) : 0.0f;
  float pitch = s.level.has_off ? (s.level.off_pitch_deg * deg2rad) : 0.0f;

  float cr = cosf(0.5f * roll);
  float sr = sinf(0.5f * roll);
  float cp = cosf(0.5f * pitch);
  float sp = sinf(0.5f * pitch);

  q[0] = cr * cp;
  q[1] = sr * cp;
  q[2] = cr * sp;
  q[3] = -sr * sp;
}

static void fc_level_cal_service(void)
{
  uint8_t sw = s.rc.level_sw ? 1u : 0u;

  if (sw != s.level.sw_last) {
    int16_t flags = (int16_t)((s.level.sw_last ? 1 : 0) |
                              (sw ? 2 : 0) |
                              (s.arm.armed ? 4 : 0) |
                              (s.rc.thr_low ? 8 : 0));
    fc_evt_push(FC_EVT_LEVEL_SW, flags, q1000(s.rc.thr),
                q10(s.att.raw_roll_deg), q10(s.att.raw_pitch_deg));
  }

  if (sw && !s.level.sw_last) {
    if (!s.arm.armed && !s.rc.arm_sw && s.rc.thr_low && s.rc.stable) {
      s.level.off_roll_deg = s.att.raw_roll_deg;
      s.level.off_pitch_deg = s.att.raw_pitch_deg;
      s.level.has_off = 1u;
      fc_evt_push(FC_EVT_LEVEL_CAL,
                  q100(s.level.off_roll_deg),
                  q100(s.level.off_pitch_deg),
                  0, 0);
    } else {
      int16_t flags = (int16_t)((s.arm.armed ? 1 : 0) |
                                (s.rc.thr_low ? 2 : 0));
      fc_evt_push(FC_EVT_LEVEL_CAL_BLOCKED, flags, q1000(s.rc.thr), 0, 0);
    }
  }

  s.level.sw_last = sw;
}

static uint8_t fc_prearm_static_ok(const float gyr[3])
{
  if (s.rc.arm_sw || !s.rc.thr_low || !s.rc.stable) return 0u;
  if (s.fs.latched != FC_FS_NONE) return 0u;

  if (fabsf(s.att.roll_deg_use) > FC_PREARM_CAL_OK_DEG) return 0u;
  if (fabsf(s.att.pitch_deg_use) > FC_PREARM_CAL_OK_DEG) return 0u;

  if (!mah.st.acc_used) return 0u;
  if (fabsf(mah.st.acc_norm_g - 1.0f) > FC_PREARM_ACC_NORM_TOL_G) return 0u;

  if (!isfinite((double)gyr[0]) || !isfinite((double)gyr[1]) || !isfinite((double)gyr[2])) return 0u;

  if (fabsf(gyr[0]) > FC_PREARM_GYRO_CAL_MAX_DPS) return 0u;
  if (fabsf(gyr[1]) > FC_PREARM_GYRO_CAL_MAX_DPS) return 0u;
  if (fabsf(gyr[2]) > FC_PREARM_GYRO_CAL_MAX_DPS) return 0u;

  return 1u;
}

static void fc_prearm_cal_service(void)
{
  uint8_t autocal_en = (uint8_t)(!s.arm.armed && !s.rc.arm_sw && s.rc.thr_low && s.rc.stable);
  imu_autocal_set_enabled(autocal_en);

  if (!s.arm.pending_ahrs_reset) {
    fc_prearm_cal_reset();
    return;
  }

  float gyr[3];
  imu_get_gyro_dps(&gyr[0], &gyr[1], &gyr[2]);

  if (!fc_prearm_static_ok(gyr)) {
    fc_prearm_cal_reset();
    return;
  }

  uint32_t now = now_ticks();
  if (s_prearm_t0 == 0u) {
    s_prearm_t0 = now;
    s_prearm_n = 0u;
    s_prearm_sum[0] = 0.0f;
    s_prearm_sum[1] = 0.0f;
    s_prearm_sum[2] = 0.0f;
  }

  s_prearm_sum[0] += gyr[0];
  s_prearm_sum[1] += gyr[1];
  s_prearm_sum[2] += gyr[2];
  if (s_prearm_n != 0xFFFFu) s_prearm_n++;

  if ((uint32_t)(now - s_prearm_t0) < sec_to_ticks(FC_PREARM_GYRO_CAL_SEC)) return;
  if (s_prearm_n < (uint16_t)FC_PREARM_GYRO_CAL_MIN_SAMPLES) return;

  float mean[3] = {
    s_prearm_sum[0] / (float)s_prearm_n,
    s_prearm_sum[1] / (float)s_prearm_n,
    s_prearm_sum[2] / (float)s_prearm_n
  };

  float old_bias[3] = {0.0f, 0.0f, 0.0f};
  if (imu_get_gyro_bias_dps(old_bias) == 0) {
    float new_bias[3] = {
      old_bias[0] + mean[0],
      old_bias[1] + mean[1],
      old_bias[2] + mean[2]
    };
    (void)imu_set_gyro_bias_dps(new_bias);
  }

  float q0[4];
  fc_q_from_level_offset(q0);
  mahony_reset(&mah, q0);

  s.arm.pending_ahrs_reset = 0u;
  fc_prearm_cal_reset();

  fc_evt_push(FC_EVT_PREARM_CAL, FC_PREARM_CAL_DONE,
              q1000(mean[0]), q1000(mean[1]), q1000(mean[2]));
  fc_evt_push(FC_EVT_AHRS_RESET,
              q10(s.att.roll_deg_use),
              q10(s.att.pitch_deg_use),
              0, 0);
}

void fc_arm_ahrs_reset_service(void)
{
  fc_level_cal_service();
  fc_prearm_cal_service();
}

fc_step_ret_t fc_arm_update(void)
{
	uint8_t was_armed = s.arm.armed;
  int arm_sw  = (int)s.rc.arm_sw;
  int thr_low = (int)s.rc.thr_low;

  uint8_t level_ok = fc_is_level_ok();

  /* ====== 解锁状态机 ====== */
  if (!s.arm.inited) {
    s.arm.inited     = 1;
    s.arm.arm_last   = (uint8_t)arm_sw;
    s.arm.last_print = -1;

    /* 上电若 arm 已经是 1：必须先拨到 0 一次 */
    s.arm.need_disarm_first = (arm_sw != 0);
		
		s.arm.pending_ahrs_reset = 1;   //上电强制先完成“预解锁自动校准”
    fc_cut_disarmed();
    return FC_STEP_STOP;
  }

  /* 上电门闩：没见到 disarm 之前永远不允许解锁 */
  if (s.arm.need_disarm_first) {
    if (arm_sw == 0) {
      s.arm.need_disarm_first = 0;
      s.arm.arm_last = 0;
      fc_clear_failsafe_latch();   /* 里面会 cut 电机 + reset */
      return FC_STEP_STOP;
    }
    fc_cut_disarmed();
    return FC_STEP_STOP;
  }

  /* 只有“真实 disarm”才清锁存：arm=0 持续一段时间 + thr_low + level_ok + RC有效 */
  static uint32_t disarm_hold_t0 = 0;

  if (arm_sw == 0 && thr_low && level_ok && s.rc.stable) {
    uint32_t tnow = now_ticks();
    if (disarm_hold_t0 == 0) disarm_hold_t0 = tnow;

    if ((uint32_t)(tnow - disarm_hold_t0) > sec_to_ticks(0.300f)) {
      if (s.fs.latched != FC_FS_NONE) {
        fc_clear_failsafe_latch();
      }
    }
  } else {
    disarm_hold_t0 = 0;
  }

  /* arm 低：强制未解锁 */
	if (arm_sw == 0) {
    if (was_armed) {
      s.arm.pending_ahrs_reset = 1;   //本次从解锁→上锁，要求下次起飞前再校准
    }
    s.arm.armed = 0;
    motors_set_armed(0);
  }
  /* ====== 翻车后必须先完成 AHRS reset，才允许再解锁（只变化打印） ====== */
  enum { ARM_BLK_NONE = 0, ARM_BLK_WAIT_RESET = 1, ARM_BLK_HARD_BLOCK = 2 };

  uint8_t blk_state =
    (!s.arm.pending_ahrs_reset) ? ARM_BLK_NONE :
    (arm_sw == 0)               ? ARM_BLK_WAIT_RESET :
                                 ARM_BLK_HARD_BLOCK;

  static uint8_t last_blk_state = 0xFF;
  if (blk_state != last_blk_state) {
    last_blk_state = blk_state;
    fc_evt_push(FC_EVT_ARM_BLOCK, (int16_t)blk_state, 0, 0, 0);
  }

  if (blk_state == ARM_BLK_HARD_BLOCK) {
    fc_cut_disarmed();
    return FC_STEP_STOP;
  }

  /* 只在 0->1 上升沿尝试解锁 */
  uint8_t arm_rise = (s.arm.arm_last == 0 && arm_sw == 1);

  if (!s.arm.armed &&
      arm_rise &&
      thr_low &&
      s.rc.stable &&
      (s.fs.latched == FC_FS_NONE) &&
      level_ok) {
    s.arm.armed = 1;
    motors_set_armed(1);
    fc_reset_controller_states();
				
		fc_evt_push(FC_EVT_MODE,
              (int16_t)(s.rc.mode_sw ? 1 : 0),   // a: 0=ACRO, 1=ANGLE
              (int16_t)FC_MODE_REASON_ARM,       // b: reason
              0, 0);		
  }
	
  /* 只在这里更新一次 */
  s.arm.arm_last = (uint8_t)arm_sw;

  if ((int)s.arm.armed != (int)s.arm.last_print) {
    s.arm.last_print = (int8_t)s.arm.armed;

    int16_t thr1000 = q1000(s.rc.thr);
    int16_t fsbits  = (int16_t)(s.fs.latched & 0x7);

    fc_evt_push(FC_EVT_ARM_STATE,
                (int16_t)arm_sw,
                (int16_t)s.arm.armed,
                thr1000,
                (int16_t)((thr_low ? 1:0) | (level_ok ? 2 : 0) | (fsbits<<2)));
  }

  /* arm 仍然为 1，但存在锁存：强制切电机，等用户 disarm 清锁存 */
  if (arm_sw && (s.fs.latched != FC_FS_NONE)) {
    fc_cut_disarmed();
    return FC_STEP_STOP;
  }

  /* 未解锁：立刻停电机（后面控制不用算了） */
  if (!s.arm.armed) {
    fc_cut_disarmed();
    return FC_STEP_STOP;
  }

  return FC_STEP_CONTINUE;
}
