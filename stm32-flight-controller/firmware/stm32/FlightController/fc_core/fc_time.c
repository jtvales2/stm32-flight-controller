#include "fc_time.h"

#include "fc_context.h"   // s, fc_emergency_stop, fc_cut_disarmed, FC_FS_IMU
#include "fc_cfg.h"         // FC_DT_* 常量
#include "fc_evt.h"
#include "timebase.h"       // now_ticks(), ticks_to_sec()
#include <math.h>

fc_step_ret_t fc_time_update_from_gyro(float dt_raw_sec)
{
  // 关键：每帧刷新“最后活着”的时间点，watchdog 用它判断卡死
  s.dt.last_ticks = now_ticks();

  // 每帧唯一编号：给“同一帧 double entry 检测”等用
  s.dt.frame_id++;

  s.dt.dt_raw = dt_raw_sec;

  // 原始 dt 非法：直接用 nominal，并且跳过 I/D（避免除 0 / 瞬时炸裂）
  if (!isfinite((double)dt_raw_sec) || dt_raw_sec <= 0.0f) {
    s.dt.dt      = FC_DT_NOM_SEC;
    s.dt.skip_id = 1u;
    return FC_STEP_CONTINUE;
  }

  // failsafe：dt 太大说明 IMU/调度链路卡了
  if (dt_raw_sec > FC_DT_FAILSAFE_SEC) {
    int32_t ms = (int32_t)(dt_raw_sec * 1000.0f);
    if (ms < 0) ms = 0;
    if (ms > 32767) ms = 32767;

    fc_evt_push(FC_EVT_IMU_HEALTH,
                FC_IMU_HEALTH_DT_STALL,
                (int16_t)ms,
                (int16_t)(s.arm.armed ? 1 : 0),
                0);

    if (s.arm.armed) fc_emergency_stop(FC_FS_IMU);
    else             fc_cut_disarmed();
    return FC_STEP_STOP;
  }

  // skip_id：dt 异常偏大时（比如偶发丢帧），控制器 I/D 不更新
  uint8_t skip = (dt_raw_sec > FC_DT_MAX_SEC) ? 1u : 0u;

  // clamp 到控制器接受的范围
  float dt = dt_raw_sec;
  if (dt < FC_DT_MIN_SEC) dt = FC_DT_MIN_SEC;
  if (dt > FC_DT_MAX_SEC) dt = FC_DT_MAX_SEC;

  s.dt.dt      = dt;
  s.dt.skip_id = skip;

  return FC_STEP_CONTINUE;
}

void fc_time_watchdog_1khz(void)
{
  // 只在解锁时看门狗才有意义
  if (!s.arm.armed) return;

  uint32_t last = s.dt.last_ticks;
  if (last == 0) return;

  uint32_t now = now_ticks();
  float age = ticks_to_sec((uint32_t)(now - last));
  if (age > FC_DT_FAILSAFE_SEC) {
    fc_emergency_stop(FC_FS_IMU);
  }
}
