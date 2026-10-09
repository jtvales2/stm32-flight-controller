#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 电调脉宽范围（单位：us） */
#define ARM_MIN_US   1000U
#define ARM_MAX_US   2000U

/* 初始化电机 PWM（开启 TIM3 CH1~4，并打最低油门） */
void motors_init(void);

/* 写入 4 个归一化电机指令（0.0~1.0） */
void motors_write(const float m[4]);
void motors_set_armed(uint8_t armed);

/* Direct test/calibration output. This bypasses normal ARM state on purpose. */
void motors_write_test(const float m[4]);

#ifdef __cplusplus
}
#endif
