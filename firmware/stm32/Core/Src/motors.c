#include "motors.h"
#include "tim.h"                // extern TIM_HandleTypeDef htim3;
#include "stm32f4xx_hal.h"
#include "main.h"            // 如果需要 Error_Handler 声明
#include "fc_cfg.h"

static uint8_t s_motors_armed = 0;

/* ===== us -> ticks cache (TIM3) ===== */
static uint32_t s_cnt_hz     = 1000000u;   // TIM3 counter clock (Hz)
static uint16_t s_arr_ticks  = 0xFFFFu;    // ARR (ticks)
static uint16_t s_min_ticks  = 1000u;      // ARM_MIN_US -> ticks
static uint16_t s_max_ticks  = 2000u;      // ARM_MAX_US -> ticks
static uint16_t s_stop_ticks = 1000u;      // stop pulse (ticks)

static uint32_t tim3_timer_clk_hz(void)
{
    RCC_ClkInitTypeDef clk = {0};
    uint32_t flash = 0;
    HAL_RCC_GetClockConfig(&clk, &flash);

    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    // STM32F4：APB1 分频 != 1 时，定时器时钟 = PCLK1 * 2
    return (clk.APB1CLKDivider == RCC_HCLK_DIV1) ? pclk1 : (pclk1 * 2u);
}

static uint16_t us_to_ticks_sat(uint32_t us)
{
    // ticks = us * cnt_hz / 1e6 (四舍五入)
    uint64_t t = (uint64_t)us * (uint64_t)s_cnt_hz + 500000ull;
    uint32_t ticks = (uint32_t)(t / 1000000ull);

    if (ticks > 0xFFFFu) ticks = 0xFFFFu;
    if (ticks > s_arr_ticks) ticks = s_arr_ticks;
    return (uint16_t)ticks;
}

static void motors_timebase_init(void)
{
    uint32_t timclk = tim3_timer_clk_hz();
    uint32_t psc    = (uint32_t)(htim3.Instance->PSC);

    s_cnt_hz    = timclk / (psc + 1u);
    s_arr_ticks     = (uint16_t)(htim3.Instance->ARR);

    s_min_ticks  = us_to_ticks_sat(ARM_MIN_US);
    s_max_ticks  = us_to_ticks_sat(ARM_MAX_US);
    if (s_max_ticks < s_min_ticks) s_max_ticks = s_min_ticks;

    s_stop_ticks = s_min_ticks;
}

static uint16_t motor_norm_to_ccr(float x)
{
    if (x < 0.0f) x = 0.0f;
    if (x > 1.0f) x = 1.0f;

    float span = (float)(s_max_ticks - s_min_ticks);
    uint32_t ticks = (uint32_t)((float)s_min_ticks + x * span + 0.5f);

    if (ticks > s_arr_ticks) ticks = s_arr_ticks;
    return (uint16_t)ticks;
}

static inline void motors_write_stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, s_stop_ticks);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, s_stop_ticks);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, s_stop_ticks);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, s_stop_ticks);
}

static void motors_write_direct_norm(const float m[4])
{
    static const uint32_t ch[4] = {
        TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4
    };

    static const uint8_t map_ch_to_m[4] = {
        FC_MOTOR_CH1_INDEX,
        FC_MOTOR_CH2_INDEX,
        FC_MOTOR_CH3_INDEX,
        FC_MOTOR_CH4_INDEX
    };

    for (int i = 0; i < 4; i++) {
        uint8_t mi = map_ch_to_m[i];
        if (mi > 3) mi = 0;
        __HAL_TIM_SET_COMPARE(&htim3, ch[i], motor_norm_to_ccr(m[mi]));
    }
}
void motors_set_armed(uint8_t armed)
{
    s_motors_armed = armed ? 1u : 0u;
    if (!s_motors_armed) {
        motors_write_stop();
    }
}

static void motor_map_sanity_check(void)
{
    const uint8_t map[4] = {
        FC_MOTOR_CH1_INDEX, FC_MOTOR_CH2_INDEX, FC_MOTOR_CH3_INDEX, FC_MOTOR_CH4_INDEX
    };

    // 范围检查 + 唯一性检查
    uint8_t seen = 0;
    for (int i = 0; i < 4; i++) {
        if (map[i] > 3) Error_Handler();
        uint8_t bit = 1u << map[i];
        if (seen & bit) Error_Handler(); // duplicate
        seen |= bit;
    }
}

void motors_init(void)
{
    motors_timebase_init();

    motor_map_sanity_check();

    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK) Error_Handler();

    motors_set_armed(0);
}

void motors_write(const float m[4])
{
    if (!s_motors_armed) {
        motors_write_stop();
        return;
    }

    motors_write_direct_norm(m);
}

void motors_write_test(const float m[4])
{
    motors_write_direct_norm(m);
}
