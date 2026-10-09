#include "fc_att.h"

#include "fc_context.h"   // s, fc_emergency_stop, FC_FS_IMU
#include "fc_cfg.h"
#include "fc_util.h"
#include "fc_evt.h"
#include "fusion_mahony.h"

#include <math.h>

// 你的姿态滤波器实例在别处定义（main.c / imu bringup 里）
extern Mahony mah;

#ifndef FC_LEVEL_OK_DEG
#define FC_LEVEL_OK_DEG  20.0f
#endif

static inline float rad2deg(float r)
{
  return r * (180.0f / 3.14159265358979323846f);
}

fc_step_ret_t fc_att_update(void)
{
  float rr, pp, yy; // radians
  mahony_get_euler(&mah, &rr, &pp, &yy);

  float roll_deg  = rad2deg(rr);
  float pitch_deg = rad2deg(pp);

  // 先写 raw（use 在 apply_offsets 里算）
  s.att.raw_roll_deg  = roll_deg;
  s.att.raw_pitch_deg = pitch_deg;

  return FC_STEP_CONTINUE;
}

fc_step_ret_t fc_att_apply_offsets_and_check(void)
{
  float r = s.att.raw_roll_deg;
  float p = s.att.raw_pitch_deg;

  if (!isfinite((double)r) || !isfinite((double)p)) {
    fc_emergency_stop(FC_FS_IMU);
    return FC_STEP_STOP;
  }

  if (s.level.has_off) {
    r -= s.level.off_roll_deg;
    p -= s.level.off_pitch_deg;
  }

  if (!isfinite((double)r) || !isfinite((double)p)) {
    fc_emergency_stop(FC_FS_IMU);
    return FC_STEP_STOP;
  }

  s.att.roll_deg_use  = r;
  s.att.pitch_deg_use = p;

  return FC_STEP_CONTINUE;
}

uint8_t fc_is_level_ok(void)
{
  return (fabsf(s.att.roll_deg_use)  < FC_LEVEL_OK_DEG) &&
         (fabsf(s.att.pitch_deg_use) < FC_LEVEL_OK_DEG);
}
