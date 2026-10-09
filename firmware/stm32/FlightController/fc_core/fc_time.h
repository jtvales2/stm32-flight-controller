#pragma once
#include <stdint.h>
#include "fc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// dt_raw_sec = 由 gyro 时间戳差得到的原始 dt（秒）
// 返回 STOP 表示 dt 触发 failsafe（已做 cut/stop 处理）
fc_step_ret_t fc_time_update_from_gyro(float dt_raw_sec);

// 可选：如果你原来在 fc_acro_watchdog_1khz 里喂 IWDG，就把实现搬到这里
void fc_time_watchdog_1khz(void);

#ifdef __cplusplus
}
#endif
