#include "fc_air.h"

#include "fc_context.h"   // s
#include "fc_cfg.h"         // FC_AIRMODE_SUPPORT + AIR_* 参数
#include "timebase.h"       // now_ticks(), sec_to_ticks()
#include <math.h>           // sqrtf, fabsf

void fc_air_reset(void)
{
  s.air.on = 0;
  s.air.ever_on = 0;      //只在 disarm/reset 清零
  s.air.takeoff_t0 = 0;
  s.air.land_t0 = 0;
}

void fc_air_update(const float gyr_dps[3])
{
  // 如果不启用 air 检测，不要把 ever_on 钉死为 0，否则 alt-hold 的门槛会永远进不去
#if !FC_AIRMODE_SUPPORT
  if (!s.arm.armed) {
    fc_air_reset();
  } else {
    s.air.on = 1;
    s.air.ever_on = 1;
    s.air.takeoff_t0 = 0;
    s.air.land_t0 = 0;
  }
  return;
#endif

  if (!s.arm.armed) {
    fc_air_reset();
    return;
  }

  uint32_t now = now_ticks();
	
	// 默认只用 RC 油门做起飞判定（避免地面 thr_smooth 滞后/抬高导致误判）
	float thr_takeoff_gate = s.rc.thr;
	
	// 只有当“控制器真的在接管油门”（例如 alt-hold active）才允许用 thr_smooth 做 gate
	if (s.ctrl.alt.active) {
    if (s.ctrl.thr_smooth > thr_takeoff_gate) thr_takeoff_gate = s.ctrl.thr_smooth;
	}

  if (!s.air.on) {
    // ---- Takeoff detect ----
    if (thr_takeoff_gate > AIR_TAKEOFF_THR) {
      if (s.air.takeoff_t0 == 0) s.air.takeoff_t0 = now;

      if ((uint32_t)(now - s.air.takeoff_t0) >= sec_to_ticks(AIR_TAKEOFF_HOLD_SEC)) {
        s.air.on = 1;
        s.air.ever_on = 1;          // 起飞后锁存（本次解锁周期内保持）
        s.air.takeoff_t0 = 0;
        s.air.land_t0 = 0;
      }
    } else {
      s.air.takeoff_t0 = 0;
    }
  } else {
    // ---- Land detect ----
    float gmag = sqrtf(gyr_dps[0]*gyr_dps[0] + gyr_dps[1]*gyr_dps[1] + gyr_dps[2]*gyr_dps[2]);

    uint8_t lvl_ok =
      (fabsf(s.att.roll_deg_use)  < AIR_LAND_MAX_TILT_DEG) &&
      (fabsf(s.att.pitch_deg_use) < AIR_LAND_MAX_TILT_DEG);

    //落地判定：以“驾驶员真收底 + 动作小 + 姿态水平”为主
    //   不要用 thr_gate=max(rc,thr_smooth) 去卡落地（thr_smooth 可能因为控制器还在托而一直不低）
    uint8_t land_cond =
      (s.rc.thr <= THR_LOW_GATE) &&
      (gmag < AIR_LAND_MAX_GYR_DPS) &&
      lvl_ok;

    if (land_cond) {
      if (s.air.land_t0 == 0) s.air.land_t0 = now;

      if ((uint32_t)(now - s.air.land_t0) >= sec_to_ticks(AIR_LAND_HOLD_SEC)) {
        s.air.on = 0;
        s.air.takeoff_t0 = 0;
        s.air.land_t0 = 0;
        // 注意：ever_on 不清（直到 disarm/reset 才清）
      }
    } else {
      s.air.land_t0 = 0;
    }
  }
}
