#pragma once

/* ===================== compile-time switches ===================== */
#define BEAM_TEST_STAND     0
#define DIRECT_MIXER_TEST   0

/* RC channels (0-based)
 * CH6 is a 3-position flight-mode switch:
 *   low  = ACRO/rate
 *   mid  = ANGLE/self-level
 *   high = ANGLE + ALT_HOLD
 * CH7 = level calibration request
 * CH8 = motor tool / ESC calibration request
 */
#ifndef FC_CFG_MODE_CH
#define FC_CFG_MODE_CH      5   // CH6, 3-position
#endif

#ifndef FC_CFG_LEVEL_CH
#define FC_CFG_LEVEL_CH     6   // CH7
#endif

#ifndef FC_CFG_ESC_CAL_CH
#define FC_CFG_ESC_CAL_CH   7   // CH8
#endif

#ifndef FC_MODE_LOW_MAX_US
#define FC_MODE_LOW_MAX_US   1300u
#endif

#ifndef FC_MODE_HIGH_MIN_US
#define FC_MODE_HIGH_MIN_US  1700u
#endif
/* ===================== motor test / ESC calibration ===================== */
#define FC_MOTOR_TEST_ENABLE        0
#define FC_ESC_CAL_ENABLE           0

/* MOTOR_TEST entry: arm=0, CH8=1, throttle low. Yaw selects motor. */
#define FC_MOTOR_TEST_MAX           0.12f
#define FC_MOTOR_TEST_SELECT_DB     0.65f
#define FC_MOTOR_TEST_LOG_MS        500u

/* ESC_CAL entry: arm=0, CH8=1, throttle high. Toggle CH8 off after DONE. */
#define FC_ESC_CAL_HIGH_THR         0.95f
#define FC_ESC_CAL_LOW_HOLD_SEC     4.0f

/* RC */
#define RC_STABLE_AFTER_LINK_SEC   0.25f
#define SBUS_RAW_MIN_SANE          150u
#define SBUS_RAW_MAX_SANE          2047u

/* throttle gates */
#define THR_LOW_GATE   0.05f

// 预解锁 gyro bias 自动校准
#define FC_PREARM_CAL_OK_DEG          5.0f     // 校准要求的最大倾角（raw roll/pitch）
#define FC_PREARM_GYRO_CAL_SEC        1.0f     // 累计时长（秒）
#define FC_PREARM_GYRO_CAL_MAX_DPS    1.5f     // 静止判定：|gyro|阈值
#define FC_PREARM_ACC_NORM_TOL_G      0.05f    // 静止判定：||acc|-1g|阈值
#define FC_PREARM_GYRO_CAL_MIN_SAMPLES 200     // 最小样本数（防止dt异常时样本太少）

/* ===================== idle throttle ===================== */
#define FC_IDLE_ENABLE      1
#define FC_IDLE_THR         0.10f
#define FC_IDLE_RAMP_SEC    0.40f

/* ===================== yaw strategy ===================== */
#define YAW_I_ENABLE_THR      0.20f
#define YAW_BLEND_THR         0.25f
#define YAW_I_DECAY_TAU_SEC   0.30f
#define YAW_CENTER_DB         (YAW_DEADBAND)

#ifndef YAW_DBG_EVERY_MS
#define YAW_DBG_EVERY_MS      2000u
#endif

/* RC deadband */
#define RP_DEADBAND           0.03f   // roll/pitch deadband（3%起步）
#define YAW_DEADBAND          0.05f

/* ===================== rate limits ===================== */
#define ACRO_MAX_RATE_ROLL_DPS    300.0f
#define ACRO_MAX_RATE_PITCH_DPS   300.0f
#define ACRO_MAX_RATE_YAW_DPS     120.0f

/* ===================== gains / limits ===================== */
#if BEAM_TEST_STAND
  #define ACRO_KP_ROLL       0.43f
  #define ACRO_KP_PITCH      0.0f
  #define ACRO_KP_YAW        0.0f

  #define ACRO_KD_ROLL       0.025f
  #define ACRO_KD_PITCH      0.0f
  #define ACRO_KD_YAW        0.0f

  #define ACRO_KI_YAW        0.0f
  #define ACRO_YAW_I_MAX     0.0f

  #define ACRO_THR_MAX       0.30f
  #define ACRO_MOTOR_MAX     0.50f
#else
  #define ACRO_KP_ROLL       0.43f
  #define ACRO_KP_PITCH      0.36f
  #define ACRO_KP_YAW        0.3f

  #define ACRO_KD_ROLL       0.025f
  #define ACRO_KD_PITCH      0.017f
  #define ACRO_KD_YAW        0.00f

  #define ACRO_KI_YAW        0.001f
  #define ACRO_YAW_I_MAX     0.2f

  #define ACRO_THR_MAX       1.0f
  #define ACRO_MOTOR_MAX     1.0f
#endif

/* ===================== angle outer loop ===================== */
#define ANGLE_MAX_ROLL_DEG       30.0f
#define ANGLE_MAX_PITCH_DEG      30.0f
#define ANGLE_KP_ROLL            2.3f
#define ANGLE_KP_PITCH           2.3f

/* ===================== timing ===================== */
#define FC_DT_NOM_SEC         0.001f
#define FC_DT_MIN_SEC         0.0005f
#define FC_DT_MAX_SEC         0.0100f
#define FC_DT_FAILSAFE_SEC    0.020f

/* ===================== gyro / IMU sanity ===================== */
#define GYR_FS_DPS            1000.0f
#define GYR_ABS_MAX_DPS       (GYR_FS_DPS * 1.2f)

/* Accel can be absent briefly; long gyro-only attitude is unsafe for this FC. */
#define FC_IMU_ACC_STALE_SEC        0.030f
#define FC_IMU_ACC_STALE_LATCH_SEC  0.120f

/* ===================== D-term LPF / throttle smoothing ===================== */
#define D_LPF_TAU_SEC         0.004f
#define THR_TAU_UP_SEC        0.025f
#define THR_TAU_DOWN_SEC      0.008f

/* ===================== tilt failsafe ===================== */
#define TILT_CUTOFF_DEG   60.0f
#define TILT_TIME_SEC     0.200f
#define TILT_THR_MIN      0.10f

/* ===================== airmode auto (in-air detect) ===================== */
#define FC_AIRMODE_SUPPORT          1
#define AIR_TAKEOFF_THR             0.25f
#define AIR_TAKEOFF_HOLD_SEC        0.35f
#define AIR_LAND_THR                0.10f
#define AIR_LAND_HOLD_SEC           0.60f
#define AIR_LAND_MAX_GYR_DPS        40.0f
#define AIR_LAND_MAX_TILT_DEG       18.0f

/* ===================== Motor mapping ===================== */
/* mixer 输出固定顺序：0=FR, 1=RR, 2=RL, 3=FL */
#define FC_MOTOR_FR   0
#define FC_MOTOR_RR   1
#define FC_MOTOR_RL   2
#define FC_MOTOR_FL   3

/* 物理 PWM 通道 -> 逻辑电机下标（改接线只改这里）
 * 默认：TIM3 CH1->FR, CH2->RR, CH3->RL, CH4->FL
 */
#define FC_MOTOR_CH1_INDEX   FC_MOTOR_FR
#define FC_MOTOR_CH2_INDEX   FC_MOTOR_RR
#define FC_MOTOR_CH3_INDEX   FC_MOTOR_RL
#define FC_MOTOR_CH4_INDEX   FC_MOTOR_FL

// ===== BARO health =====
#define FC_BARO_STALE_MS   150u   // 100~200ms 都行；你现在 baro ~70Hz，120ms 很安全

// ===== ALT HOLD (cascaded z -> vz -> thr) =====
// Conservative first-flight logic: enter only when the craft is already steady.
#define FC_ALT_ENTER_SEC            0.80f
#define FC_ALT_ENTER_MAX_VZ_MPS     0.25f
#define FC_ALT_ENTER_THR_MIN        0.30f
#define FC_ALT_ENTER_THR_MAX        0.70f
#define FC_ALT_ENTER_MIN_Z          0.80f

// stick -> vz_cmd. Throttle stick becomes climb/descent speed command.
#define FC_ALT_VZ_MAX        0.35f    // m/s
#define FC_ALT_VZ_DB         0.12f    // deadband (thr unit around captured mid)

// outer loop: height error -> vz correction
#define FC_ALT_KZ_TO_VZ      0.30f    // (m) -> (m/s)
#define FC_ALT_VZ_CORR_MAX   0.35f    // m/s

// inner loop: vz error -> throttle correction
#define FC_ALT_KP_VZ         0.14f
#define FC_ALT_KI_VZ         0.00f    // keep 0 for first-flight tuning

#define FC_ALT_I_MAX         0.30f
#define FC_ALT_THR_CORR_MAX  0.25f    // limit correction around captured hover throttle
#define FC_ALT_TILT_COS_MIN  0.60f    // tilt compensation safety floor
#define FC_ALT_Z_MIN        -2.0f
#define FC_ALT_Z_MAX        50.0f
