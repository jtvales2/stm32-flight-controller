#pragma once
#include "fc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// 翻车后 reset 服务（只在 disarm + thr_low + level_ok 时执行）
void fc_arm_ahrs_reset_service(void);

// 解锁状态机（含 ARM_BLOCK/ARM_STATE 事件），返回 STOP 表示本帧结束（切电机/阻塞等）
fc_step_ret_t fc_arm_update(void);

#ifdef __cplusplus
}
#endif
