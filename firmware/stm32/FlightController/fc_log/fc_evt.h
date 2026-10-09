#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * FC event bus: SPSC（单生产者+单消费者）
 * 规则：
 *  - 控制环/ISR 只 push（不 printf）
 *  - 主循环 log_poll 里 pop 后再 printf
 */

typedef enum {
  FC_EVT_ARM_STATE = 1,
  FC_EVT_ARM_BLOCK,
  FC_EVT_LEVEL_SW,
  FC_EVT_LEVEL_CAL,
  FC_EVT_LEVEL_CAL_BLOCKED,
  FC_EVT_TILT_LATCH,
  FC_EVT_AHRS_RESET,
  FC_EVT_WARN_CTRL2X,
  FC_EVT_FAILSAFE,
  FC_EVT_YAW_DBG,
	FC_EVT_MODE,
  FC_EVT_PREARM_CAL,
  FC_EVT_IMU_HEALTH,
  FC_EVT_ALT_ENTER,
    FC_EVT_ALT_ENTER_BLOCK,
  FC_EVT_ALT_EXIT,
  FC_EVT_ALT_DBG,
  FC_EVT_ALT_DBG2,
  FC_EVT_IMU_DMA_TIMEOUT,
  FC_EVT_BARO_ZERO,
  FC_EVT_MOTOR_TEST,
  FC_EVT_ESC_CAL,
} fc_evt_id_t;

#define FC_MODE_REASON_INIT    0
#define FC_MODE_REASON_SWITCH  1
#define FC_MODE_REASON_ARM     2

#define FC_PREARM_CAL_DONE     1

#define FC_IMU_HEALTH_GYRO_PTR     1
#define FC_IMU_HEALTH_GYRO_NAN     2
#define FC_IMU_HEALTH_GYRO_RANGE   3
#define FC_IMU_HEALTH_ACC_STALE    4
#define FC_IMU_HEALTH_DT_STALL     5
#define FC_IMU_HEALTH_DROP_HARDSTOP 6

#define FC_ALT_EXIT_BARO_STALE     1
#define FC_ALT_EXIT_SWITCH_OFF     2
#define FC_ALT_EXIT_GATE_BAD       3
#define FC_ALT_EXIT_POGO           4

#define FC_ALT_ENTER_BLOCK_RC_UNSTABLE  1
#define FC_ALT_ENTER_BLOCK_VZ           2
#define FC_ALT_ENTER_BLOCK_THR          3
#define FC_ALT_ENTER_BLOCK_LOW_ALT      4

#define FC_MOTOR_TEST_ENTER        1
#define FC_MOTOR_TEST_SELECT       2
#define FC_MOTOR_TEST_OUTPUT       3
#define FC_MOTOR_TEST_EXIT         4

#define FC_ESC_CAL_ENTER           1
#define FC_ESC_CAL_HIGH            2
#define FC_ESC_CAL_LOW             3
#define FC_ESC_CAL_DONE            4
#define FC_ESC_CAL_ABORT           5

#define FC_MOTOR_TOOL_STOP_COMBO   1
#define FC_MOTOR_TOOL_STOP_ARM     2

typedef struct {
  uint32_t t;        /* timestamp (now_ticks) */
  uint16_t id;       /* fc_evt_id_t */
  int16_t  a, b, c, d;
} fc_evt_t;

void     fc_evt_init(void);

/* ISR-safe push: caller supplies timestamp */
void     fc_evt_push_isr(fc_evt_id_t id, int16_t a, int16_t b, int16_t c, int16_t d, uint32_t t);

/* Normal push: stamps with now_ticks() internally */
void     fc_evt_push(fc_evt_id_t id, int16_t a, int16_t b, int16_t c, int16_t d);

/* Pop one event. Returns 1 on success, 0 if empty. */
int      fc_evt_pop(fc_evt_t *out);

/* Fetch-and-clear drop counter (events dropped due to full queue). */
uint32_t fc_evt_take_drop_count(void);

#ifdef __cplusplus
}
#endif


