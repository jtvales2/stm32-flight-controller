#pragma once
#include <stdint.h>

// TIM5 1MHz timebase
uint32_t now_ticks(void);      // us (32-bit, wraps ~71.6 min)
uint64_t now_ticks64(void);    // us (64-bit, optional)

// ticks(us) <-> seconds
float    ticks_to_sec(uint32_t dt_ticks); // us -> sec
uint32_t sec_to_ticks(float s);           // sec -> us

// TIM5 overflow ISR hook (call from HAL_TIM_PeriodElapsedCallback when TIM5)
void     timebase_tim5_overflow_isr(void);
