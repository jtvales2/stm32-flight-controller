#include "fc_core.h"
#include "stm32f4xx.h"
#include "fc_throttle.h"
#include "fc_context.h"
#include "fc_evt.h"
#include "fc_control.h"
#include "fc_cfg.h"
#include "fc_util.h"
#include "fc_log.h"
#include "fc_rc.h"
#include "fc_att.h"
#include "fc_time.h"
#include "fc_air.h"
#include "fc_arm.h"
#include "fc_fs.h"
#include "fc_mode.h"
#include "timebase.h"
#include "sync_pair.h"
#include "motors.h"

#include <math.h>

fc_state_t s;

static uint32_t s_acc_stale_t0 = 0u;
static uint8_t  s_imu_evt_last_reason = 0u;
static uint32_t s_imu_evt_last_ticks = 0u;

static void fc_imu_health_event(uint8_t reason, int16_t b, int16_t c, int16_t d)
{
    uint32_t now = now_ticks();
    if ((reason != s_imu_evt_last_reason) ||
        (s_imu_evt_last_ticks == 0u) ||
        ((uint32_t)(now - s_imu_evt_last_ticks) >= sec_to_ticks(1.0f))) {
        s_imu_evt_last_reason = reason;
        s_imu_evt_last_ticks = now;
        fc_evt_push(FC_EVT_IMU_HEALTH, reason, b, c, d);
    }
}

static fc_step_ret_t fc_imu_health_fault(uint8_t reason, int16_t b, int16_t c, int16_t d)
{
    fc_imu_health_event(reason, b, c, d);

    if (s.arm.armed) {
        fc_emergency_stop(FC_FS_IMU);
    } else {
        fc_cut_disarmed();
    }

    return FC_STEP_STOP;
}

static fc_step_ret_t fc_imu_health_check(const float gyr_dps[3])
{
    if (gyr_dps == 0) {
        return fc_imu_health_fault(FC_IMU_HEALTH_GYRO_PTR, 0, 0, 0);
    }

    float gx = gyr_dps[0];
    float gy = gyr_dps[1];
    float gz = gyr_dps[2];

    if (!isfinite((double)gx) || !isfinite((double)gy) || !isfinite((double)gz)) {
        return fc_imu_health_fault(FC_IMU_HEALTH_GYRO_NAN, 0, 0, 0);
    }

    float max_abs = fabsf(gx);
    if (fabsf(gy) > max_abs) max_abs = fabsf(gy);
    if (fabsf(gz) > max_abs) max_abs = fabsf(gz);

    if (max_abs > GYR_ABS_MAX_DPS) {
        return fc_imu_health_fault(FC_IMU_HEALTH_GYRO_RANGE, q10(max_abs), 0, 0);
    }

    const IMU_SyncPairStats *sp = imu_sync_pair_stats();
    uint32_t acc_age = sp ? sp->last_acc_age_ticks : 0xFFFFFFFFu;
    uint32_t acc_age_limit = sec_to_ticks(FC_IMU_ACC_STALE_SEC);

    if (acc_age == 0xFFFFFFFFu || acc_age > acc_age_limit) {
        uint32_t now = now_ticks();
        if (s_acc_stale_t0 == 0u) {
            s_acc_stale_t0 = now;
        }

        if ((uint32_t)(now - s_acc_stale_t0) >= sec_to_ticks(FC_IMU_ACC_STALE_LATCH_SEC)) {
            uint32_t age_ms_u = 32767u;
            if (acc_age != 0xFFFFFFFFu) {
                age_ms_u = (uint32_t)(ticks_to_sec(acc_age) * 1000.0f);
                if (age_ms_u > 32767u) age_ms_u = 32767u;
            }
            return fc_imu_health_fault(FC_IMU_HEALTH_ACC_STALE,
                                       (int16_t)age_ms_u,
                                       q1000(s.rc.thr),
                                       (int16_t)(s.arm.armed ? 1 : 0));
        }
    } else {
        s_acc_stale_t0 = 0u;
    }

    return FC_STEP_CONTINUE;
}

void fc_core_baro_feed(uint8_t valid, uint32_t ts_ticks,
                       int32_t press_pa, int32_t temp_centi,
                       float alt_rel_m, float vz_mps)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (!valid) {
        s.baro.valid = 0u;
    } else {
        s.baro.ts_ticks   = ts_ticks;
        s.baro.press_pa   = press_pa;
        s.baro.temp_centi = temp_centi;
        s.baro.alt_rel_m  = alt_rel_m;
        s.baro.vz_mps     = vz_mps;

        s.baro.stale_pending = 0u;
        s.baro.valid      = 1u;   // 最后发布
    }

    __set_PRIMASK(primask);
}

static void fc_motor_tool_write_all(float x)
{
    float m[4] = {x, x, x, x};
    motors_set_armed(0u);
    motors_write_test(m);
}

static void fc_motor_tool_write_one(uint8_t sel, float x)
{
    float m[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    if (sel > 3u) sel = 0u;
    if (x < 0.0f) x = 0.0f;
    if (x > FC_MOTOR_TEST_MAX) x = FC_MOTOR_TEST_MAX;
    m[sel] = x;
    motors_set_armed(0u);
    motors_write_test(m);
}

static uint8_t fc_motor_tool_sw_active(void)
{
    return (uint8_t)(s.rc.valid && s.rc.stable && s.rc.esc_cal_sw);
}

static uint8_t fc_motor_tool_combo(void)
{
    return (uint8_t)(fc_motor_tool_sw_active() && !s.rc.arm_sw);
}

static fc_step_ret_t fc_motor_tool_update(float dt)
{
    (void)dt;

#if (FC_MOTOR_TEST_ENABLE || FC_ESC_CAL_ENABLE)
    enum {
        MOTOR_TOOL_NONE = 0,
        MOTOR_TOOL_TEST = 1,
        MOTOR_TOOL_ESC_HIGH = 2,
        MOTOR_TOOL_ESC_LOW = 3
    };

    static uint8_t  tool = MOTOR_TOOL_NONE;
    static uint8_t  test_sel = 0u;
    static int8_t   yaw_latch = 0;
    static uint32_t phase_t0 = 0u;
    static uint32_t log_t0 = 0u;
    static float    last_thr_log = -1.0f;
    static uint8_t  inhibit_until_sw_off = 0u;

    uint32_t now = now_ticks();
    uint8_t sw_active = fc_motor_tool_sw_active();
    uint8_t combo = (uint8_t)(sw_active && !s.rc.arm_sw);

    if (inhibit_until_sw_off) {
        fc_motor_tool_write_all(0.0f);
        if (!sw_active) {
            inhibit_until_sw_off = 0u;
            return FC_STEP_CONTINUE;
        }
        return FC_STEP_STOP;
    }

    if (sw_active && s.rc.arm_sw) {
        fc_motor_tool_write_all(0.0f);
        if (tool == MOTOR_TOOL_TEST) {
            fc_evt_push(FC_EVT_MOTOR_TEST, FC_MOTOR_TEST_EXIT,
                        (int16_t)test_sel, FC_MOTOR_TOOL_STOP_ARM, 0);
        } else if (tool != MOTOR_TOOL_NONE) {
            fc_evt_push(FC_EVT_ESC_CAL, FC_ESC_CAL_ABORT,
                        FC_MOTOR_TOOL_STOP_ARM, 0, 0);
        }
        tool = MOTOR_TOOL_NONE;
        yaw_latch = 0;
        return FC_STEP_STOP;
    }

#if FC_MOTOR_TEST_ENABLE
    if (tool == MOTOR_TOOL_TEST) {
        if (!combo) {
            fc_motor_tool_write_all(0.0f);
            fc_evt_push(FC_EVT_MOTOR_TEST, FC_MOTOR_TEST_EXIT,
                        (int16_t)test_sel, FC_MOTOR_TOOL_STOP_COMBO, 0);
            tool = MOTOR_TOOL_NONE;
            yaw_latch = 0;
            return FC_STEP_STOP;
        }

        int8_t yaw_dir = 0;
        if (s.rc.yaw > FC_MOTOR_TEST_SELECT_DB) yaw_dir = 1;
        else if (s.rc.yaw < -FC_MOTOR_TEST_SELECT_DB) yaw_dir = -1;

        if (yaw_dir == 0) {
            yaw_latch = 0;
        } else if (yaw_latch == 0) {
            if (yaw_dir > 0) test_sel = (uint8_t)((test_sel + 1u) & 3u);
            else             test_sel = (uint8_t)((test_sel + 3u) & 3u);
            yaw_latch = yaw_dir;
            fc_evt_push(FC_EVT_MOTOR_TEST, FC_MOTOR_TEST_SELECT,
                        (int16_t)test_sel, 0, 0);
        }

        float thr = s.rc.thr;
        if (thr < 0.0f) thr = 0.0f;
        if (thr > FC_MOTOR_TEST_MAX) thr = FC_MOTOR_TEST_MAX;
        fc_motor_tool_write_one(test_sel, thr);

        uint32_t log_period = sec_to_ticks((float)FC_MOTOR_TEST_LOG_MS / 1000.0f);
        if (log_period == 0u) log_period = sec_to_ticks(0.5f);
        if (log_t0 == 0u || (uint32_t)(now - log_t0) >= log_period ||
            fabsf(thr - last_thr_log) >= 0.02f) {
            log_t0 = now;
            last_thr_log = thr;
            fc_evt_push(FC_EVT_MOTOR_TEST, FC_MOTOR_TEST_OUTPUT,
                        (int16_t)test_sel, q1000(thr), 0);
        }

        return FC_STEP_STOP;
    }
#endif

#if FC_ESC_CAL_ENABLE
    if (tool == MOTOR_TOOL_ESC_HIGH || tool == MOTOR_TOOL_ESC_LOW) {
        if (!combo) {
            fc_motor_tool_write_all(0.0f);
            fc_evt_push(FC_EVT_ESC_CAL, FC_ESC_CAL_ABORT,
                        FC_MOTOR_TOOL_STOP_COMBO, 0, 0);
            tool = MOTOR_TOOL_NONE;
            return FC_STEP_STOP;
        }

        if (tool == MOTOR_TOOL_ESC_HIGH) {
            fc_motor_tool_write_all(1.0f);
            if (s.rc.thr_low) {
                tool = MOTOR_TOOL_ESC_LOW;
                phase_t0 = now;
                fc_evt_push(FC_EVT_ESC_CAL, FC_ESC_CAL_LOW, 0,
                            (int16_t)(FC_ESC_CAL_LOW_HOLD_SEC * 10.0f), 0);
            }
            return FC_STEP_STOP;
        }

        fc_motor_tool_write_all(0.0f);
        if ((uint32_t)(now - phase_t0) >= sec_to_ticks(FC_ESC_CAL_LOW_HOLD_SEC)) {
            fc_evt_push(FC_EVT_ESC_CAL, FC_ESC_CAL_DONE, 0, 0, 0);
            tool = MOTOR_TOOL_NONE;
            inhibit_until_sw_off = 1u;
        }
        return FC_STEP_STOP;
    }
#endif

#if FC_ESC_CAL_ENABLE
    if (combo && s.rc.thr >= FC_ESC_CAL_HIGH_THR) {
        tool = MOTOR_TOOL_ESC_HIGH;
        phase_t0 = now;
        fc_motor_tool_write_all(1.0f);
        fc_evt_push(FC_EVT_ESC_CAL, FC_ESC_CAL_ENTER, q1000(s.rc.thr), 0, 0);
        fc_evt_push(FC_EVT_ESC_CAL, FC_ESC_CAL_HIGH, 1000, 0, 0);
        return FC_STEP_STOP;
    }
#endif

#if FC_MOTOR_TEST_ENABLE
    if (combo && s.rc.thr_low) {
        tool = MOTOR_TOOL_TEST;
        test_sel = 0u;
        yaw_latch = 0;
        log_t0 = 0u;
        last_thr_log = -1.0f;
        fc_motor_tool_write_one(test_sel, 0.0f);
        fc_evt_push(FC_EVT_MOTOR_TEST, FC_MOTOR_TEST_ENTER,
                    (int16_t)test_sel, q1000(FC_MOTOR_TEST_MAX), 0);
        return FC_STEP_STOP;
    }
#endif

    if (sw_active) {
        fc_motor_tool_write_all(0.0f);
        return FC_STEP_STOP;
    }

#else
    (void)0;
#endif

    return FC_STEP_CONTINUE;
}
void fc_core_step(const float gyr_dps[3], float dt_sec)
{
    if (fc_time_update_from_gyro(dt_sec) != FC_STEP_CONTINUE) return;
    if (fc_imu_health_check(gyr_dps) != FC_STEP_CONTINUE) return;
	
    float   dt      = s.dt.dt;
    uint8_t skip_id = s.dt.skip_id;
	
    if (fc_rc_update() != FC_STEP_CONTINUE) return;
    if (fc_motor_tool_update(dt) != FC_STEP_CONTINUE) return;
    if (fc_att_update() != FC_STEP_CONTINUE) return;
    if (fc_att_apply_offsets_and_check() != FC_STEP_CONTINUE) return;
    fc_arm_ahrs_reset_service();
	if (fc_arm_update() != FC_STEP_CONTINUE) return;
	fc_ctrl_idle_ramp_update(dt);
	fc_ctrl_thr_update(dt, skip_id);
	fc_air_update(gyr_dps);
	
	fc_mode_update();
	
    if (fc_fs_tilt_update() != FC_STEP_CONTINUE) return;
    if (fc_ctrl_run(gyr_dps, dt, skip_id) != FC_STEP_CONTINUE) return;
}

static inline void fc_baro_stale_check_1khz(void)
{
    if (!s.baro.valid) return;

    uint32_t now = now_ticks();                 // 1MHz -> us tick
    uint32_t age_us = (uint32_t)(now - s.baro.ts_ticks);
    uint32_t stale_us = (uint32_t)FC_BARO_STALE_MS * 1000u;

    if (s.baro.ts_ticks == 0u || age_us > stale_us) {
        s.baro.stale_pending = 1u;
    }
}

void fc_core_watchdog_1khz(void)
{
  fc_time_watchdog_1khz();
	fc_baro_stale_check_1khz();
}

void fc_core_service_async(void)
{
    uint8_t baro_stale;
    uint8_t alt_was_active;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    baro_stale = s.baro.stale_pending;
    s.baro.stale_pending = 0u;
    __set_PRIMASK(primask);

    if (!baro_stale) {
        return;
    }

    alt_was_active = s.ctrl.alt.active;

    s.baro.valid = 0u;
    s.baro.ts_ticks = 0u;
    s.baro.alt_rel_m = 0.0f;
    s.baro.vz_mps = 0.0f;

    s.ctrl.alt.active = 0u;
    s.ctrl.alt.enter_t = 0.0f;
    s.ctrl.alt.i_term = 0.0f;
    s.ctrl.alt.thr_out = 0.0f;
    s.ctrl.alt.was_sw = s.rc.alt_sw;

    if (alt_was_active) {
        fc_evt_push(FC_EVT_ALT_EXIT, FC_ALT_EXIT_BARO_STALE, 0, 0, 0);
    }
}

void fc_core_log_poll(void)
{
  fc_log_poll();
}

