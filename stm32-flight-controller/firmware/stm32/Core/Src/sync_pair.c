#include "sync_pair.h"
#include "autocal.h"
#include "timebase.h"
#include <string.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef struct {
    IMU_BMI088_FE *fe;
    Mahony        *mah;
    rb_vec_t      *acc_rb;
    rb_vec_t      *gyr_rb;

    float acc_fs;
    float gyr_fs;

    /* 最新且“不超前于当前gyro时间”的 accel */
    stamped_vec3_t last_A;
    uint8_t        have_A;

    /* 调试：这里用“|G.ts - last_A.ts|（实际就是age）”来替代原先配对残差 */
    uint32_t last_pair_resid_ticks;

    /* 上一次 gyro 的时间戳，用来算 dt */
    uint32_t last_g_ts;
} SyncPairCtx;

static SyncPairCtx SP;
static volatile uint32_t s_g_seq = 0;
static volatile uint32_t s_g_ts  = 0;
static volatile float    s_g_dps[3] = {0};

/* === 对外：姿态 / 角速度 === */
void imu_get_attitude_euler(float *roll, float *pitch, float *yaw)
{
    if (!SP.mah) {
        if (roll)  *roll  = 0.0f;
        if (pitch) *pitch = 0.0f;
        if (yaw)   *yaw   = 0.0f;
        return;
    }
    mahony_get_euler(SP.mah, roll, pitch, yaw);
}

uint32_t imu_get_gyro_seq(void) { return s_g_seq; }
uint32_t imu_get_gyro_ts(void)  { return s_g_ts;  }

void imu_get_gyro_dps(float *gx, float *gy, float *gz)
{
    // 如果还没发布过 gyro（比如刚启动），返回 0
    if (s_g_seq == 0) {
        if (gx) *gx = 0.0f;
        if (gy) *gy = 0.0f;
        if (gz) *gz = 0.0f;
        return;
    }

    uint32_t s1, s2;
    float x, y, z;

    // 读两次 seq，确保读到的是同一次发布的三轴
    do {
        s1 = s_g_seq;
        __DMB();
        x = s_g_dps[0];
        y = s_g_dps[1];
        z = s_g_dps[2];
        __DMB();
        s2 = s_g_seq;
    } while (s1 != s2);

    if (gx) *gx = x;
    if (gy) *gy = y;
    if (gz) *gz = z;
}

/* 绑定依赖 */
void imu_sync_pair_bind(IMU_BMI088_FE *fe,
                        Mahony *mah,
                        rb_vec_t *acc_rb,
                        rb_vec_t *gyr_rb)
{
    memset(&SP, 0, sizeof(SP));
    SP.fe     = fe;
    SP.mah    = mah;
    SP.acc_rb = acc_rb;
    SP.gyr_rb = gyr_rb;
}

/* 设置运行时采样率（Hz） */
void imu_sync_pair_set_fs(float acc_fs, float gyr_fs)
{
    SP.acc_fs = acc_fs;
    SP.gyr_fs = gyr_fs;
}

void imu_sync_pair_reset(void)
{
    // Do NOT touch bound pointers or fs; only clear dynamic state.
    memset(&SP.last_A, 0, sizeof(SP.last_A));
    SP.have_A = 0u;

    SP.last_pair_resid_ticks = 0xFFFFFFFFu;
    SP.last_g_ts = 0u;

    // Also reset legacy getter publish state.
    s_g_seq = 0u;
    s_g_ts  = 0u;
    s_g_dps[0] = 0.0f;
    s_g_dps[1] = 0.0f;
    s_g_dps[2] = 0.0f;
}

static inline int32_t ts_diff(uint32_t a, uint32_t b) { return (int32_t)(a - b); }

/* accel 允许“滞后”多久还能用于校正（太旧就不用，Mahony 自动变成纯积分） */
static inline uint32_t acc_age_limit_ticks(void)
{
    float Ta = (SP.acc_fs > 0.0f) ? (1.0f / SP.acc_fs) : 0.00125f; // 默认≈800Hz
    float max_age = 3.0f * Ta;     // 允许落后约 3 个 accel 周期
    if (max_age < 0.003f) max_age = 0.003f;  // >=3ms
    if (max_age > 0.010f) max_age = 0.010f;  // <=10ms
    return sec_to_ticks(max_age);
}

int imu_sync_pair_step_one(float gyr_dps_out[3], float *dt_sec_out)
{
    if (!SP.mah || !SP.gyr_rb || !SP.acc_rb) return IMU_SP_E_BADPTR;

    stamped_vec3_t G;
    if (!rbv_pop_oldest(SP.gyr_rb, &G)) return IMU_SP_NO_DATA;

    // 时间戳必须单调递增（否则 dt 会乱）
    if (SP.last_g_ts != 0u && ts_diff(G.ts, SP.last_g_ts) <= 0) {
        SP.last_g_ts = G.ts;  // 轻量自愈：重新对齐
        return IMU_SP_E_TS_ORDER;
    }

    // dt：跟随 gyro 时间线
    float dt;
    if (SP.last_g_ts == 0u) {
        float fs = (SP.gyr_fs > 0.0f) ? SP.gyr_fs : 1000.0f;
        dt = 1.0f / fs;
    } else {
        dt = ticks_to_sec(G.ts - SP.last_g_ts);
    }
    SP.last_g_ts = G.ts;

    //把最新 gyro 存一份给 legacy getter 用
    s_g_seq++; __DMB();
    s_g_ts = G.ts;
    s_g_dps[0] = G.v[0];
    s_g_dps[1] = G.v[1];
    s_g_dps[2] = G.v[2];
    __DMB(); s_g_seq++;

    // 关键：只消耗 ts <= G.ts 的 accel，缓存最后一个到 last_A（复用）
    stamped_vec3_t Aold;
    while (rbv_peek_oldest(SP.acc_rb, &Aold, NULL)) {
        if (ts_diff(Aold.ts, G.ts) <= 0) {
            rbv_pop_oldest(SP.acc_rb, &SP.last_A);
            SP.have_A = 1u;
        } else {
            break;
        }
    }

    float ax = 0.f, ay = 0.f, az = 0.f;
    uint32_t resid = 0xFFFFFFFFu;

    if (SP.have_A) {
        resid = (uint32_t)(G.ts - SP.last_A.ts);   // 因为 last_A.ts <= G.ts
        if (resid <= acc_age_limit_ticks()) {
            ax = SP.last_A.v[0];
            ay = SP.last_A.v[1];
            az = SP.last_A.v[2];
            maybe_static_calibrate_gz(&SP.last_A, &G);
        }
    }
    SP.last_pair_resid_ticks = resid;

    mahony_update_6dof(SP.mah,
                       G.v[0], G.v[1], G.v[2],
                       ax, ay, az,
                       dt);

    if (gyr_dps_out) {
        gyr_dps_out[0] = G.v[0];
        gyr_dps_out[1] = G.v[1];
        gyr_dps_out[2] = G.v[2];
    }
    if (dt_sec_out) *dt_sec_out = dt;

    return IMU_SP_OK;
}

int imu_sync_pair_fuse_some(int Nmax)
{
    int fused = 0;
    if (!SP.mah || !SP.gyr_rb || !SP.acc_rb) return 0;

    while (fused < Nmax) {
        int r = imu_sync_pair_step_one(NULL, NULL);
        if (r == IMU_SP_NO_DATA) break;   // 没数据：正常退出
        if (r < 0) continue;              // 错误：丢掉这一帧 gyro，继续
        fused++;
    }
    return fused;
}

const IMU_SyncPairStats* imu_sync_pair_stats(void){
  static IMU_SyncPairStats s;
	s.last_acc_age_ticks = SP.last_pair_resid_ticks; // 或者你也把 SP 里字段改名
    return &s;
}
