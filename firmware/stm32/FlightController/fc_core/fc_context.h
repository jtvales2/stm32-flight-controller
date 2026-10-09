#pragma once

#include <stdint.h>
#include "sbus.h"

typedef enum {
  FC_FS_NONE = 0,
  FC_FS_LINK = 1u << 0,
  FC_FS_TILT = 1u << 1,
  FC_FS_IMU  = 1u << 2,
} fc_failsafe_reason_t;

// ==== 运行态
typedef struct {
  struct {
    sbus_frame_t fr;
    uint32_t     last_rx_ticks;   // timebase ticks (us)
    uint32_t     timeout_ticks;   // SBUS 超时阈值 ticks（比如 0.2s）
    uint8_t      valid;
    uint32_t     link_up_ticks;   // RC 从 invalid->valid 的时刻
    uint8_t      stable;          // RC 已稳定一段时间

    float   thr, roll, pitch, yaw;
    uint8_t arm_sw;
		uint8_t mode_sw;    // CH6 3-pos: low=ACRO, mid/high=ANGLE
    uint8_t thr_low;
		uint8_t alt_sw;   // CH6 high: 1=Alt-hold
    uint8_t level_sw; // CH7: level calibration request
    uint8_t esc_cal_sw; // CH8: motor tool / ESC calibration request
  } rc;

  struct {
    float raw_roll_deg, raw_pitch_deg;
    float roll_deg_use, pitch_deg_use;
  } att;

  struct {
    float   off_roll_deg, off_pitch_deg;
    uint8_t has_off;
    uint8_t sw_last;
  } level;

  struct {
    uint8_t armed;
    uint8_t arm_last;
    uint8_t inited;
    int8_t  last_print;
    uint8_t need_disarm_first;
    uint8_t pending_ahrs_reset;
  } arm;

  struct {
    uint32_t latched;          // bitmask: FC_FS_LINK / FC_FS_TILT / FC_FS_IMU
    uint32_t tilt_over_ticks;  // ticks
    uint32_t tilt_time_ticks;  // 超角持续阈值 ticks（比如 0.2s）
  } fs;

  struct {
    uint32_t last_ticks;  // watchdog 用
    float    dt;     
    float    dt_raw;      // 原始 dt
    uint8_t  skip_id;
    uint32_t frame_id;
	} dt;


  struct {
    float thr_smooth;
    float yaw_i_term;

    float roll_rate_prev, roll_d_lpf;
    uint8_t roll_d_inited;

    float pitch_rate_prev, pitch_d_lpf;
    uint8_t pitch_d_inited;

    // ===== ALT HOLD (outer loop) =====
    struct {
        uint8_t active;     // 当前是否处于 alt-hold
        uint8_t was_sw;     // 上一帧开关状态（做边沿）
        float   z_sp_m;     // 高度设定（相对高度，m）
        float   thr_mid;    // 捕获“当前油门位置”为中位
        float   thr_hover;  // 捕获悬停油门基准
        float   i_term;     // 高度积分项
        float   thr_out;    // 输出到 u.throttle 的油门（可用于日志）
			  float   enter_t;     //ENTER保护期倒计时（sec）
    } alt;
	} ctrl;

  struct {
    uint8_t  on;         // 1=离地（启发式）
    uint8_t  ever_on;    // 1=本次解锁后曾经离地（锁存）
    uint32_t takeoff_t0;
    uint32_t land_t0;
  } air;
	
  struct {
    uint8_t  valid;        // 1=baro 输出可用
    uint8_t  stale_pending; // set by 1kHz watchdog, consumed in main/control context
    uint32_t ts_ticks;     // timebase ticks(us)，用于算 age
		uint32_t rx_ms;
    int32_t  press_pa;     // 原始压力 Pa
    int32_t  temp_centi;   // 温度 0.01℃
    float    alt_rel_m;    // 相对高度（米）
    float    vz_mps;       // 垂直速度（m/s）
  } baro;
	
  struct {
    float roll_deg_cmd,  roll_deg_meas,  roll_rate_cmd,  roll_rate_meas;
    float pitch_deg_cmd, pitch_deg_meas, pitch_rate_cmd, pitch_rate_meas;
  } dbg;

  float   idle_ramp_t;
  uint8_t was_armed;
} fc_state_t;

#ifdef __cplusplus
extern "C" {
#endif
	
// ==== 全局状态（由 fc_acro.c 定义）====
extern fc_state_t s;

// ==== 由 fc_acro.c 提供，ctrl 里会调用 ====
void fc_emergency_stop(uint32_t reason);
void fc_cut_disarmed(void);
void fc_clear_failsafe_latch(void);
void fc_reset_controller_states(void);

#ifdef __cplusplus
}
#endif

