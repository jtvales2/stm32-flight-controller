#include "stm32f4xx_hal.h"
#include "tim.h"
#include "timebase.h"

extern TIM_HandleTypeDef htim5;

static volatile uint32_t s_hi32 = 0;

void timebase_tim5_overflow_isr(void)
{
    s_hi32++;
}

uint32_t now_ticks(void)
{
    // 1 tick = 1 us（低 32 位，~71 分钟回卷）
    return __HAL_TIM_GET_COUNTER(&htim5);
}

uint64_t now_ticks64(void)
{
    uint32_t hi1 = s_hi32;
    uint32_t lo  = __HAL_TIM_GET_COUNTER(&htim5);
    uint32_t hi2 = s_hi32;

    if (hi2 != hi1) {
        // 溢出 ISR 夹在中间：以 hi2 为准，重读 lo
        hi1 = hi2;
        lo  = __HAL_TIM_GET_COUNTER(&htim5);
    }

    // 溢出已发生但 ISR 还没跑到：补偿 +1
    if (__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_UPDATE) != RESET) {
        hi1 += 1u;
        // 可选：再读一次 lo（防止刚好卡在回卷边界）
        lo = __HAL_TIM_GET_COUNTER(&htim5);
    }

    return ((uint64_t)hi1 << 32) | (uint64_t)lo;
}

float ticks_to_sec(uint32_t dt_ticks)
{
    return (float)dt_ticks * 1e-6f;
}

uint32_t sec_to_ticks(float s)
{
    if (s <= 0.0f) return 0u;
    float us = s * 1e6f;
    if (us > 4294967295.0f) us = 4294967295.0f;
    return (uint32_t)(us + 0.5f);
}
