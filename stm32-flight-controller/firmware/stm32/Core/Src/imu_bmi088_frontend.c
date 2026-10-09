#include "imu_bmi088_frontend.h"
#include <math.h>

/* BMI088 accel sensor_time 为 24-bit 计数。LSB 时间按数据手册设置。
 * 常见为 39.0625us/LSB (1/25600s)。如果你的版本不同，改下面这个宏。*/
#ifndef BMI088_ST_LSB_SEC
#define BMI088_ST_LSB_SEC (1.0f/25600.0f)   /* 约 39.0625 us */
#endif
#define BMI088_ST_MASK 0xFFFFFFu

/* ====================== 内部工具 ====================== */
// 3x3 * vec（行主序）
static inline void mat3x3_mul_vec_flat(const float R[9], const float v[3], float out[3]) {
    out[0] = R[0]*v[0] + R[1]*v[1] + R[2]*v[2];
    out[1] = R[3]*v[0] + R[4]*v[1] + R[5]*v[2];
    out[2] = R[6]*v[0] + R[7]*v[1] + R[8]*v[2];
}

// 3x3 * 3x3（行主序）
static inline void mat3x3_mul_mat_flat(const float A[9], const float B[9], float C[9]) {
    C[0]=A[0]*B[0]+A[1]*B[3]+A[2]*B[6];
    C[1]=A[0]*B[1]+A[1]*B[4]+A[2]*B[7];
    C[2]=A[0]*B[2]+A[1]*B[5]+A[2]*B[8];
    C[3]=A[3]*B[0]+A[4]*B[3]+A[5]*B[6];
    C[4]=A[3]*B[1]+A[4]*B[4]+A[5]*B[7];
    C[5]=A[3]*B[2]+A[4]*B[5]+A[5]*B[8];
    C[6]=A[6]*B[0]+A[7]*B[3]+A[8]*B[6];
    C[7]=A[6]*B[1]+A[7]*B[4]+A[8]*B[7];
    C[8]=A[6]*B[2]+A[7]*B[5]+A[8]*B[8];
}

static void biquad_design_lpf(float fs, float fc, float Q,
                              float *b0,float *b1,float *b2,float *a1,float *a2)
{
    if (fc <= 0.0f || fs <= 0.0f) {
        *b0 = 1.0f; *b1 = *b2 = *a1 = *a2 = 0.0f; return;
    }
    const float two_pi = 6.28318530717958647692f;  // 2*pi，避免依赖 M_PI
    const float w0 = two_pi * fc / fs;
    const float cosw0 = (float)cos(w0), sinw0 = (float)sin(w0);
    const float alpha = sinw0/(2.0f*Q);

    float b0n = (1.0f - cosw0)*0.5f;
    float b1n =  1.0f - cosw0;
    float b2n = (1.0f - cosw0)*0.5f;
    float a0  =  1.0f + alpha;
    float a1n = -2.0f * cosw0;
    float a2n =  1.0f - alpha;

    *b0 = b0n/a0; *b1 = b1n/a0; *b2 = b2n/a0;
    *a1 = a1n/a0; *a2 = a2n/a0;
}

static inline float biquad_proc(float x, float b0,float b1,float b2,float a1,float a2, float *z1, float *z2)
{
    // Transposed Direct Form II
    float y = b0*x + *z1;
    *z1 = b1*x - a1*y + *z2;
    *z2 = b2*x - a2*y;
    return y;
}

static void biquad_set_all(IMU_BMI088_FE *fe)
{
    const float fsA = fe->st.runtime_acc_sample_hz > 0 ? fe->st.runtime_acc_sample_hz : 100.0f;
    const float fsG = fe->st.runtime_gyr_sample_hz > 0 ? fe->st.runtime_gyr_sample_hz : 100.0f;
    const float Q   = 0.70710678f; // 巴特沃斯
    for (int i=0;i<3;i++) {
        biquad_design_lpf(fsA, fe->cfg.acc_lpf_cutoff_hz, Q,
                          &fe->lpf_a[i].b0,&fe->lpf_a[i].b1,&fe->lpf_a[i].b2,&fe->lpf_a[i].a1,&fe->lpf_a[i].a2);
        fe->lpf_a[i].z1 = fe->lpf_a[i].z2 = 0.0f;

        biquad_design_lpf(fsG, fe->cfg.gyr_lpf_cutoff_hz, Q,
                          &fe->lpf_g[i].b0,&fe->lpf_g[i].b1,&fe->lpf_g[i].b2,&fe->lpf_g[i].a1,&fe->lpf_g[i].a2);
        fe->lpf_g[i].z1 = fe->lpf_g[i].z2 = 0.0f;
    }
}

static float odr_to_sample_hz_acc(uint8_t odr)
{
    switch (odr) {
    case BMI08X_ACCEL_ODR_12_5_HZ: return 12.5f;
    case BMI08X_ACCEL_ODR_25_HZ:   return 25.0f;
    case BMI08X_ACCEL_ODR_50_HZ:   return 50.0f;
    case BMI08X_ACCEL_ODR_100_HZ:  return 100.0f;
    case BMI08X_ACCEL_ODR_200_HZ:  return 200.0f;
    case BMI08X_ACCEL_ODR_400_HZ:  return 400.0f;
    case BMI08X_ACCEL_ODR_800_HZ:  return 800.0f;
    case BMI08X_ACCEL_ODR_1600_HZ: return 1600.0f;
    default: return 0.0f;
    }
}

static float odr_to_sample_hz_gyr(uint8_t bw_odr_enum)
{
    /* 你的驱动把 ODR 编在 BW 枚举名里（见 *_BW_xxx_ODR_yyy_HZ） */
    switch (bw_odr_enum) {
    case BMI08X_GYRO_BW_532_ODR_2000_HZ: return 2000.0f;
    case BMI08X_GYRO_BW_230_ODR_2000_HZ: return 2000.0f;
    case BMI08X_GYRO_BW_116_ODR_1000_HZ: return 1000.0f;
    case BMI08X_GYRO_BW_47_ODR_400_HZ:   return 400.0f;
    case BMI08X_GYRO_BW_23_ODR_200_HZ:   return 200.0f;
    case BMI08X_GYRO_BW_12_ODR_100_HZ:   return 100.0f;
    case BMI08X_GYRO_BW_64_ODR_200_HZ:   return 200.0f;
    case BMI08X_GYRO_BW_32_ODR_100_HZ:   return 100.0f;
    default: return 0.0f;
    }
}

static float acc_g_per_lsb_from_range(uint8_t r)
{
    /* BMI088 加计：3/6/12/24 g。你的库默认 24g（自检时） */
    float full_g;
    switch (r) {
    case BMI088_ACCEL_RANGE_3G:  full_g = 3.0f;  break;
    case BMI088_ACCEL_RANGE_6G:  full_g = 6.0f;  break;
    case BMI088_ACCEL_RANGE_12G: full_g = 12.0f; break;
    case BMI088_ACCEL_RANGE_24G: full_g = 24.0f; break;
    default: full_g = 24.0f; break;
    }
    return full_g / 32768.0f;
}

static float gyr_dps_per_lsb_from_range(uint8_t r)
{
    float full;
    switch (r) {
    case BMI08X_GYRO_RANGE_2000_DPS: full = 2000.0f; break;
    case BMI08X_GYRO_RANGE_1000_DPS: full = 1000.0f; break;
    case BMI08X_GYRO_RANGE_500_DPS:  full = 500.0f;  break;
    case BMI08X_GYRO_RANGE_250_DPS:  full = 250.0f;  break;
    case BMI08X_GYRO_RANGE_125_DPS:  full = 125.0f;  break;
    default: full = 1000.0f; break;
    }
    return full / 32768.0f;
}

static void set_identity9(float R[9])
{
    R[0]=1; R[1]=0; R[2]=0;
    R[3]=0; R[4]=1; R[5]=0;
    R[6]=0; R[7]=0; R[8]=1;
}

static inline int is_near_zero9(const float R[9], float eps)
{
    float s = 0.0f;
    for (int i = 0; i < 9; i++) s += fabsf(R[i]);
    return s < eps;
}

/* 同步模式枚举选择（尽量贴近 cfg ODR）*/
static uint8_t choose_sync_mode(uint8_t acc_odr, uint8_t gyr_odr)
{
    const float fa = odr_to_sample_hz_acc(acc_odr);
    const float fg = odr_to_sample_hz_gyr(gyr_odr);
    const float f  = (fa>0 && fg>0) ? ((fa < fg) ? fa : fg) : 0.0f;
    if      (f >= 1500.0f) return BMI08X_ACCEL_DATA_SYNC_MODE_2000HZ;
    else if (f >=  700.0f) return BMI08X_ACCEL_DATA_SYNC_MODE_1000HZ;
    else if (f >=  300.0f) return BMI08X_ACCEL_DATA_SYNC_MODE_400HZ;
    else return 0; // 不支持的，同步不开
}

/* ====================== 前端实现 ====================== */

static int apply_cfg_lowlevel(IMU_BMI088_FE *fe, const IMU_BMI088_Config *cfg)
{
    fe->dev->accel_cfg.odr   = cfg->acc_odr;
    fe->dev->accel_cfg.bw    = BMI08X_ACCEL_BW_OSR4;   // 带宽最宽，延迟最小
    fe->dev->accel_cfg.range = cfg->acc_range;
    fe->st.last_bosch_rslt = bmi08a_set_meas_conf(fe->dev);
    if (fe->st.last_bosch_rslt != BMI08X_OK)
        return fe->st.last_bosch_rslt;

    fe->dev->gyro_cfg.odr   = cfg->gyr_odr;
    // 仍需给一个合法 bw，但不指望芯片做截止；选与你 odr 匹配但带宽较宽的枚举
    fe->dev->gyro_cfg.bw    = cfg->gyr_odr;
    fe->dev->gyro_cfg.range = cfg->gyr_range;
    fe->st.last_bosch_rslt = bmi08g_set_meas_conf(fe->dev);
    if (fe->st.last_bosch_rslt != BMI08X_OK)
        return fe->st.last_bosch_rslt;

    /* 如需同步，选最接近的官方模式并配置 */
    if (cfg->use_sync) {
        struct bmi08x_data_sync_cfg sync = {0};
        sync.mode = choose_sync_mode(cfg->acc_odr, cfg->gyr_odr);
        if (sync.mode == 0) {
            fe->st.flags |= IMU_BMI088_ST_SYNC_REQUESTED_BUT_UNAVAIL;
        } else {
            fe->st.last_bosch_rslt = bmi088_configure_data_synchronization(sync, fe->dev);
            if (fe->st.last_bosch_rslt != BMI08X_OK)
                return fe->st.last_bosch_rslt;
        }
    }

    return BMI08X_OK;
}

static int readback_and_finalize(IMU_BMI088_FE *fe)
{
    /* 读回实际寄存器值（驱动会填回 dev->*_cfg）*/
    int8_t r1 = bmi08a_get_meas_conf(fe->dev);
    int8_t r2 = bmi08g_get_meas_conf(fe->dev);
    fe->st.last_bosch_rslt = (r1 != BMI08X_OK) ? r1 : r2;
    if (fe->st.last_bosch_rslt != BMI08X_OK)
        return fe->st.last_bosch_rslt;

    /* ODR → 采样率（真实运行时）*/
    fe->st.runtime_acc_sample_hz = odr_to_sample_hz_acc(fe->dev->accel_cfg.odr);
    fe->st.runtime_gyr_sample_hz = odr_to_sample_hz_gyr(fe->dev->gyro_cfg.odr);

    /* 根据“读回的 range”算 LSB→物理量 */
    fe->acc_g_per_lsb    = acc_g_per_lsb_from_range(fe->dev->accel_cfg.range);
    fe->gyr_dps_per_lsb  = gyr_dps_per_lsb_from_range(fe->dev->gyro_cfg.range);

    /* 配 LPF（按 runtime_*_sample_hz）*/
    biquad_set_all(fe);

		/* 校验一致性：先清这四个 mismatch 位，再按本次读回结果置位 */
		fe->st.flags &= ~( IMU_BMI088_ST_ACC_ODR_MISMATCH
											| IMU_BMI088_ST_ACC_RANGE_MISMATCH
											| IMU_BMI088_ST_GYR_ODR_MISMATCH
											| IMU_BMI088_ST_GYR_RANGE_MISMATCH );
		
		if (fe->dev->accel_cfg.odr   != fe->cfg.acc_odr)
				fe->st.flags |= IMU_BMI088_ST_ACC_ODR_MISMATCH;

		if (fe->dev->accel_cfg.range != fe->cfg.acc_range)
				fe->st.flags |= IMU_BMI088_ST_ACC_RANGE_MISMATCH;

		if (fe->dev->gyro_cfg.odr    != fe->cfg.gyr_odr)
				fe->st.flags |= IMU_BMI088_ST_GYR_ODR_MISMATCH;

		if (fe->dev->gyro_cfg.range  != fe->cfg.gyr_range)
				fe->st.flags |= IMU_BMI088_ST_GYR_RANGE_MISMATCH;

    return BMI08X_OK;
}

// 输入：单位化的 v_from, v_to；输出：R，使得 R*v_from = v_to
static void rot_from_to(const float v_from[3], const float v_to[3], float R[9])
{
    const float vx = v_from[1]*v_to[2] - v_from[2]*v_to[1];
    const float vy = v_from[2]*v_to[0] - v_from[0]*v_to[2];
    const float vz = v_from[0]*v_to[1] - v_from[1]*v_to[0];
    const float c  = v_from[0]*v_to[0] + v_from[1]*v_to[1] + v_from[2]*v_to[2];
    const float s2 = vx*vx + vy*vy + vz*vz;

    // 默认 I
    R[0]=1; R[1]=0; R[2]=0;
    R[3]=0; R[4]=1; R[5]=0;
    R[6]=0; R[7]=0; R[8]=1;

    // 近同向/反向的退化处理
    if (s2 < 1e-12f) {
        if (c < 0.0f) {
            const float ax = (fabsf(v_from[0]) < 0.9f) ? 1.0f : 0.0f;
            const float ay = (fabsf(v_from[0]) < 0.9f) ? 0.0f : 1.0f;
            float ux = v_from[1]*0.0f - v_from[2]*ay;
            float uy = v_from[2]*ax   - v_from[0]*0.0f;
            float uz = v_from[0]*ay   - v_from[1]*ax;
            const float un = 1.0f/sqrtf(ux*ux+uy*uy+uz*uz);
            ux*=un; uy*=un; uz*=un;
            const float xx=ux*ux, yy=uy*uy, zz=uz*uz, xy=ux*uy, xz=ux*uz, yz=uy*uz;
            R[0]=1-2*(yy+zz); R[1]=2*xy;        R[2]=2*xz;
            R[3]=2*xy;        R[4]=1-2*(xx+zz); R[5]=2*yz;
            R[6]=2*xz;        R[7]=2*yz;        R[8]=1-2*(xx+yy);
        }
        return;
    }

    // R = c*I + [v]_x + ((1-c)/s^2) * (v v^T)
    const float beta = (1.0f - c) / s2;

    // c*I
    R[0]=c; R[1]=0; R[2]=0;
    R[3]=0; R[4]=c; R[5]=0;
    R[6]=0; R[7]=0; R[8]=c;

    // + [v]_x
    R[1] -= vz; R[2] += vy;
    R[3] += vz; R[5] -= vx;
    R[6] -= vy; R[7] += vx;

    // + beta * (v v^T)
    R[0] += beta*vx*vx; R[1] += beta*vx*vy; R[2] += beta*vx*vz;
    R[3] += beta*vy*vx; R[4] += beta*vy*vy; R[5] += beta*vy*vz;
    R[6] += beta*vz*vx; R[7] += beta*vz*vy; R[8] += beta*vz*vz;
}

// 采样 samples 帧加速度（机体系），把平均向量旋到 z 轴；左乘更新 R_sb
int IMU_BMI088_FE_AutoLevel(IMU_BMI088_FE *fe, uint16_t samples, uint16_t inter_ms)
{
    if (!fe) return BMI08X_E_NULL_PTR;
    if (samples == 0) samples = 800;

    float ab[3], gb[3], T, sum[3] = {0,0,0};
    for (uint16_t i=0; i<samples; i++) {
        if (IMU_BMI088_FE_Read(fe, ab, gb, &T) != BMI08X_OK) return fe->st.last_bosch_rslt;
        sum[0]+=ab[0]; sum[1]+=ab[1]; sum[2]+=ab[2];
        if (fe->dev && fe->dev->delay_ms && inter_ms) fe->dev->delay_ms(inter_ms);
    }
    const float inv = 1.0f / (float)samples;
    float m[3] = { sum[0]*inv, sum[1]*inv, sum[2]*inv };

    /* === 新增护栏：只有静止近 1g 才更新安装矩阵 === */
    float norm = sqrtf(m[0]*m[0] + m[1]*m[1] + m[2]*m[2]);
    if (norm < 0.85f || norm > 1.15f) {
        /* 不改 R_sb，直接返回（也可以改成返回一个自定义错误码） */
        return BMI08X_OK;
        /* 若你更想“报错”而不是静默跳过，就用：
           #ifdef BMI08X_E_INVALID_INPUT
               return BMI08X_E_INVALID_INPUT;
           #else
               return -120; // IMU_BMI088_FE: not stationary
           #endif
        */
    }
    const float n = 1.0f / (norm + 1e-20f);
    float from[3] = { m[0]*n, m[1]*n, m[2]*n };
    const float to[3]   = { 0.0f, 0.0f, 1.0f };  // NED: 向下

    float R_level[9], R_new[9];
    rot_from_to(from, to, R_level);
    mat3x3_mul_mat_flat(R_level, fe->cfg.R_sb, R_new);      // 左乘：R_sb' = R_level * R_sb
    memcpy(fe->cfg.R_sb, R_new, sizeof(R_new));
    return BMI08X_OK;
}


/* ====================== 公共 API ====================== */

int IMU_BMI088_FE_Init(IMU_BMI088_FE *fe, struct bmi08x_dev *dev, const IMU_BMI088_Config *cfg)
{
    if (!fe || !dev || !cfg) return BMI08X_E_NULL_PTR;

    memset(fe, 0, sizeof(*fe));
    fe->dev = dev;
    fe->cfg = *cfg;
	
	if (is_near_zero9(fe->cfg.R_sb, 1e-6f)) {
    set_identity9(fe->cfg.R_sb);
	}

    int8_t rs = apply_cfg_lowlevel(fe, cfg);
    if (rs != BMI08X_OK) { fe->st.flags |= IMU_BMI088_ST_APPLY_ERROR; return rs; }

    rs = readback_and_finalize(fe);
    if (rs != BMI08X_OK) { fe->st.flags |= IMU_BMI088_ST_APPLY_ERROR; return rs; }

    /* 初次读温度（毫摄氏度） */
    fe->st.last_bosch_rslt = bmi08a_get_sensor_temperature(fe->dev, &fe->last_temp_mC);

    return BMI08X_OK;
}

int IMU_BMI088_FE_Reconfigure(IMU_BMI088_FE *fe, const IMU_BMI088_Config *cfg)
{
    if (!fe || !cfg) return BMI08X_E_NULL_PTR;
    fe->cfg = *cfg;

    int8_t rs = apply_cfg_lowlevel(fe, cfg);
    if (rs != BMI08X_OK) { fe->st.flags |= IMU_BMI088_ST_APPLY_ERROR; return rs; }

    return readback_and_finalize(fe);
}

int IMU_BMI088_FE_AssertConfigConsistency(IMU_BMI088_FE *fe)
{
    if (!fe) return BMI08X_E_NULL_PTR;
    /* 清旧标志再做一次读回 */
    fe->st.flags &= ~(IMU_BMI088_ST_ACC_ODR_MISMATCH|IMU_BMI088_ST_ACC_RANGE_MISMATCH|
                      IMU_BMI088_ST_GYR_ODR_MISMATCH|IMU_BMI088_ST_GYR_RANGE_MISMATCH);
    return readback_and_finalize(fe);
}

void IMU_BMI088_FE_SetBias(IMU_BMI088_FE *fe, const float acc_bias_g[3], const float gyr_bias_dps[3])
{
    if (!fe) return;
    if (acc_bias_g)  memcpy(fe->cfg.acc_bias_g,  acc_bias_g,  sizeof(fe->cfg.acc_bias_g));
    if (gyr_bias_dps)memcpy(fe->cfg.gyr_bias_dps,gyr_bias_dps,sizeof(fe->cfg.gyr_bias_dps));
}

void IMU_BMI088_FE_SetTempComp(IMU_BMI088_FE *fe, uint8_t enable, float T0,
                               const float b0[3], const float b1[3], const float b2[3])
{
    if (!fe) return;
    fe->cfg.tc_enable = enable;
    fe->cfg.tc_T0 = T0;
    if (b0) memcpy(fe->cfg.tc_b0, b0, sizeof(fe->cfg.tc_b0));
    if (b1) memcpy(fe->cfg.tc_b1, b1, sizeof(fe->cfg.tc_b1));
    if (b2) memcpy(fe->cfg.tc_b2, b2, sizeof(fe->cfg.tc_b2));
}

void IMU_BMI088_FE_SetMounting(IMU_BMI088_FE *fe, const float R_sb[9])
{
    if (!fe) return;
    if (R_sb) memcpy(fe->cfg.R_sb, R_sb, sizeof(fe->cfg.R_sb)); // 9 * sizeof(float)
}

/* 一帧：读取→换算→安装矩阵→LPF→温补→去偏 */
int IMU_BMI088_FE_Read(IMU_BMI088_FE *fe, float acc_b_g[3], float gyr_b_dps[3], float *temp_C)
{
    if (!fe) return BMI08X_E_NULL_PTR;

    struct bmi08x_sensor_data a = {0}, g = {0};
    int8_t rs;

    if (fe->cfg.use_sync) {
        rs = bmi088_get_synchronized_data(&a, &g, fe->dev);
    } else {
			rs = bmi08g_get_data(&g, fe->dev);                 // 先读陀螺
      if (rs == BMI08X_OK) rs = bmi08a_get_data(&a, fe->dev); // 再读加计
    }
    fe->st.last_bosch_rslt = rs;
    if (rs != BMI08X_OK) return rs;

    /* 原始 LSB -> 传感器坐标系物理量 */
    float acc_s_g[3] = {
        a.x * fe->acc_g_per_lsb,
        a.y * fe->acc_g_per_lsb,
        a.z * fe->acc_g_per_lsb
    };
    float gyr_s_dps[3] = {
        g.x * fe->gyr_dps_per_lsb,
        g.y * fe->gyr_dps_per_lsb,
        g.z * fe->gyr_dps_per_lsb
    };

    /* 传感器->机体 */
    float acc_b[3], gyr_b[3];
    mat3x3_mul_vec_flat(fe->cfg.R_sb, acc_s_g,  acc_b);
    mat3x3_mul_vec_flat(fe->cfg.R_sb, gyr_s_dps, gyr_b);

    /* 二阶 LPF */
    acc_b[0] = biquad_proc(acc_b[0], fe->lpf_a[0].b0, fe->lpf_a[0].b1, fe->lpf_a[0].b2,
                                      fe->lpf_a[0].a1, fe->lpf_a[0].a2, &fe->lpf_a[0].z1, &fe->lpf_a[0].z2);
    acc_b[1] = biquad_proc(acc_b[1], fe->lpf_a[1].b0, fe->lpf_a[1].b1, fe->lpf_a[1].b2,
                                      fe->lpf_a[1].a1, fe->lpf_a[1].a2, &fe->lpf_a[1].z1, &fe->lpf_a[1].z2);
    acc_b[2] = biquad_proc(acc_b[2], fe->lpf_a[2].b0, fe->lpf_a[2].b1, fe->lpf_a[2].b2,
                                      fe->lpf_a[2].a1, fe->lpf_a[2].a2, &fe->lpf_a[2].z1, &fe->lpf_a[2].z2);

    gyr_b[0] = biquad_proc(gyr_b[0], fe->lpf_g[0].b0, fe->lpf_g[0].b1, fe->lpf_g[0].b2,
                                      fe->lpf_g[0].a1, fe->lpf_g[0].a2, &fe->lpf_g[0].z1, &fe->lpf_g[0].z2);
    gyr_b[1] = biquad_proc(gyr_b[1], fe->lpf_g[1].b0, fe->lpf_g[1].b1, fe->lpf_g[1].b2,
                                      fe->lpf_g[1].a1, fe->lpf_g[1].a2, &fe->lpf_g[1].z1, &fe->lpf_g[1].z2);
    gyr_b[2] = biquad_proc(gyr_b[2], fe->lpf_g[2].b0, fe->lpf_g[2].b1, fe->lpf_g[2].b2,
                                      fe->lpf_g[2].a1, fe->lpf_g[2].a2, &fe->lpf_g[2].z1, &fe->lpf_g[2].z2);

    /* 温度（毫摄氏度→摄氏度）*/
    int32_t t_mC = 0;
    if (bmi08a_get_sensor_temperature(fe->dev, &t_mC) == BMI08X_OK) {
        fe->last_temp_mC = t_mC;
    }
    const float T  = 0.001f * (float)fe->last_temp_mC;
    const float dT = T - fe->cfg.tc_T0;
    float tc[3] = {
        fe->cfg.tc_b0[0] + fe->cfg.tc_b1[0]*dT + fe->cfg.tc_b2[0]*dT*dT,
        fe->cfg.tc_b0[1] + fe->cfg.tc_b1[1]*dT + fe->cfg.tc_b2[1]*dT*dT,
        fe->cfg.tc_b0[2] + fe->cfg.tc_b1[2]*dT + fe->cfg.tc_b2[2]*dT*dT
    };

    /* 去偏（静态零偏 + 可选温补增量）*/
    gyr_b[0] -= (fe->cfg.gyr_bias_dps[0] + (fe->cfg.tc_enable ? tc[0] : 0.0f));
    gyr_b[1] -= (fe->cfg.gyr_bias_dps[1] + (fe->cfg.tc_enable ? tc[1] : 0.0f));
    gyr_b[2] -= (fe->cfg.gyr_bias_dps[2] + (fe->cfg.tc_enable ? tc[2] : 0.0f));

    acc_b[0] -= fe->cfg.acc_bias_g[0];
    acc_b[1] -= fe->cfg.acc_bias_g[1];
    acc_b[2] -= fe->cfg.acc_bias_g[2];

    /* 输出与缓存 */
    if (acc_b_g) {
        acc_b_g[0]=acc_b[0]; acc_b_g[1]=acc_b[1]; acc_b_g[2]=acc_b[2];
    }
    if (gyr_b_dps) {
        gyr_b_dps[0]=gyr_b[0]; gyr_b_dps[1]=gyr_b[1]; gyr_b_dps[2]=gyr_b[2];
    }
    if (temp_C) *temp_C = T;

		memcpy(fe->last_acc_b_g,   acc_b, sizeof(acc_b));
    memcpy(fe->last_gyr_b_dps, gyr_b, sizeof(gyr_b));

    return BMI08X_OK;
}

int IMU_BMI088_FE_ReadAccOnly(IMU_BMI088_FE *fe, float acc_b_g[3], float *temp_C)
{
    if (!fe) return BMI08X_E_NULL_PTR;

    struct bmi08x_sensor_data a = {0};
    int8_t rs = bmi08a_get_data(&a,fe->dev);
    fe->st.last_bosch_rslt = rs;
    if (rs != BMI08X_OK) return rs;

    float acc_s_g[3] = {
        a.x * fe->acc_g_per_lsb,
        a.y * fe->acc_g_per_lsb,
        a.z * fe->acc_g_per_lsb
    };

    float acc_b[3];
    mat3x3_mul_vec_flat(fe->cfg.R_sb, acc_s_g, acc_b);/* 传感器->机体 */
    acc_b[0] = biquad_proc(acc_b[0], fe->lpf_a[0].b0, fe->lpf_a[0].b1, fe->lpf_a[0].b2,
                                       fe->lpf_a[0].a1, fe->lpf_a[0].a2, &fe->lpf_a[0].z1, &fe->lpf_a[0].z2);
    acc_b[1] = biquad_proc(acc_b[1], fe->lpf_a[1].b0, fe->lpf_a[1].b1, fe->lpf_a[1].b2,
                                       fe->lpf_a[1].a1, fe->lpf_a[1].a2, &fe->lpf_a[1].z1, &fe->lpf_a[1].z2);
    acc_b[2] = biquad_proc(acc_b[2], fe->lpf_a[2].b0, fe->lpf_a[2].b1, fe->lpf_a[2].b2,
                                       fe->lpf_a[2].a1, fe->lpf_a[2].a2, &fe->lpf_a[2].z1, &fe->lpf_a[2].z2);

    acc_b[0] -= fe->cfg.acc_bias_g[0];
    acc_b[1] -= fe->cfg.acc_bias_g[1];
    acc_b[2] -= fe->cfg.acc_bias_g[2];

    if (acc_b_g) { acc_b_g[0]=acc_b[0]; acc_b_g[1]=acc_b[1]; acc_b_g[2]=acc_b[2]; }
    memcpy(fe->last_acc_b_g, acc_b, sizeof(acc_b));

    if (temp_C) {
        int32_t t_mC = 0;
        if (bmi08a_get_sensor_temperature(fe->dev, &t_mC) == BMI08X_OK) {
            fe->last_temp_mC = t_mC;
        }
        *temp_C = 0.001f * (float)fe->last_temp_mC;
    }
    return BMI08X_OK;
}

int IMU_BMI088_FE_ReadGyrOnly(IMU_BMI088_FE *fe, float gyr_b_dps[3], float *temp_C)
{
    if (!fe) return BMI08X_E_NULL_PTR;

    struct bmi08x_sensor_data g = {0};
    int8_t rs = bmi08g_get_data(&g,fe->dev);
    fe->st.last_bosch_rslt = rs;
    if (rs != BMI08X_OK) return rs;

    float gyr_s_dps[3] = {
        g.x * fe->gyr_dps_per_lsb,
        g.y * fe->gyr_dps_per_lsb,
        g.z * fe->gyr_dps_per_lsb
    };

    // 温度用于陀螺温漂补偿：从加计温度通道取最近温度（与 Read 一致）
    int32_t t_mC = 0;
    if (bmi08a_get_sensor_temperature(fe->dev, &t_mC) == BMI08X_OK) {
        fe->last_temp_mC = t_mC;
    }
    const float T  = 0.001f * (float)fe->last_temp_mC;
    const float dT = T - fe->cfg.tc_T0;
    float tc[3] = {
        fe->cfg.tc_b0[0] + fe->cfg.tc_b1[0]*dT + fe->cfg.tc_b2[0]*dT*dT,
        fe->cfg.tc_b0[1] + fe->cfg.tc_b1[1]*dT + fe->cfg.tc_b2[1]*dT*dT,
        fe->cfg.tc_b0[2] + fe->cfg.tc_b1[2]*dT + fe->cfg.tc_b2[2]*dT*dT
    }; 

    float gyr_b[3];
    mat3x3_mul_vec_flat(fe->cfg.R_sb, gyr_s_dps, gyr_b); 
    gyr_b[0] = biquad_proc(gyr_b[0], fe->lpf_g[0].b0, fe->lpf_g[0].b1, fe->lpf_g[0].b2,
                                       fe->lpf_g[0].a1, fe->lpf_g[0].a2, &fe->lpf_g[0].z1, &fe->lpf_g[0].z2);
    gyr_b[1] = biquad_proc(gyr_b[1], fe->lpf_g[1].b0, fe->lpf_g[1].b1, fe->lpf_g[1].b2,
                                       fe->lpf_g[1].a1, fe->lpf_g[1].a2, &fe->lpf_g[1].z1, &fe->lpf_g[1].z2);
    gyr_b[2] = biquad_proc(gyr_b[2], fe->lpf_g[2].b0, fe->lpf_g[2].b1, fe->lpf_g[2].b2,
                                       fe->lpf_g[2].a1,fe->lpf_g[2].a2, &fe->lpf_g[2].z1, &fe->lpf_g[2].z2);

    gyr_b[0] -= (fe->cfg.gyr_bias_dps[0] + (fe->cfg.tc_enable ? tc[0] : 0.0f));
    gyr_b[1] -= (fe->cfg.gyr_bias_dps[1] + (fe->cfg.tc_enable ? tc[1] : 0.0f));
    gyr_b[2] -= (fe->cfg.gyr_bias_dps[2] + (fe->cfg.tc_enable ? tc[2] : 0.0f));

    if (gyr_b_dps) { gyr_b_dps[0]=gyr_b[0]; gyr_b_dps[1]=gyr_b[1]; gyr_b_dps[2]=gyr_b[2]; }
    memcpy(fe->last_gyr_b_dps, gyr_b, sizeof(gyr_b));
    if (temp_C) *temp_C = T;

    return BMI08X_OK;
}

int IMU_BMI088_FE_ReadTimed(IMU_BMI088_FE *fe,
                            float acc_b_g[3], float gyr_b_dps[3], float *temp_C,
                            float *dt_sec)
{
    if (!fe) return BMI08X_E_NULL_PTR;

    /* 先拿一帧同步数据（或独立数据）；你已有封装，直接复用 */
    int8_t rs = IMU_BMI088_FE_Read(fe, acc_b_g, gyr_b_dps, temp_C);
    if (rs != BMI08X_OK) return rs;

    /* 读取 accel sensor_time（24-bit） */
    uint32_t st_now = 0;
    rs = bmi08a_get_sensor_time(fe->dev, &st_now);   /* 官方 API */
    if (rs != BMI08X_OK) return rs;                  /* :contentReference[oaicite:4]{index=4} */

    /* 计算模 2^24 的差分，得到 tick 数 */
    uint32_t dt_tick = fe->st_has_last
        ? ((st_now - fe->st_last_24) & BMI088_ST_MASK)
        : (uint32_t)( (fe->st.runtime_gyr_sample_hz > 0.f)
                        ? (uint32_t)( (1.0f/fe->st.runtime_gyr_sample_hz) / BMI088_ST_LSB_SEC )
                        : 0u );

    fe->st_last_24 = st_now;
    fe->st_has_last = 1;

    if (dt_sec) *dt_sec = (float)dt_tick * (float)BMI088_ST_LSB_SEC;
    return BMI08X_OK;
}

void IMU_BMI088_FE_ProcessAccRaw(IMU_BMI088_FE *fe,
                                 int16_t ax, int16_t ay, int16_t az,
                                 float acc_b_g[3])
{
    if (!fe) return;

    /* raw -> 传感器坐标系物理量(g) */
    float acc_s_g[3] = {
        (float)ax * fe->acc_g_per_lsb,
        (float)ay * fe->acc_g_per_lsb,
        (float)az * fe->acc_g_per_lsb
    };

    /* 传感器系 -> 机体系 */
    float acc_b[3];
    mat3x3_mul_vec_flat(fe->cfg.R_sb, acc_s_g, acc_b);

    /* LPF（沿用现有 biquad 状态） */
    acc_b[0] = biquad_proc(acc_b[0], fe->lpf_a[0].b0, fe->lpf_a[0].b1, fe->lpf_a[0].b2,
                                      fe->lpf_a[0].a1, fe->lpf_a[0].a2, &fe->lpf_a[0].z1, &fe->lpf_a[0].z2);
    acc_b[1] = biquad_proc(acc_b[1], fe->lpf_a[1].b0, fe->lpf_a[1].b1, fe->lpf_a[1].b2,
                                      fe->lpf_a[1].a1, fe->lpf_a[1].a2, &fe->lpf_a[1].z1, &fe->lpf_a[1].z2);
    acc_b[2] = biquad_proc(acc_b[2], fe->lpf_a[2].b0, fe->lpf_a[2].b1, fe->lpf_a[2].b2,
                                      fe->lpf_a[2].a1, fe->lpf_a[2].a2, &fe->lpf_a[2].z1, &fe->lpf_a[2].z2);

    /* 去 acc bias（g） */
    acc_b[0] -= fe->cfg.acc_bias_g[0];
    acc_b[1] -= fe->cfg.acc_bias_g[1];
    acc_b[2] -= fe->cfg.acc_bias_g[2];

    /* 输出 + 缓存 */
    if (acc_b_g) {
        acc_b_g[0] = acc_b[0];
        acc_b_g[1] = acc_b[1];
        acc_b_g[2] = acc_b[2];
    }
    memcpy(fe->last_acc_b_g, acc_b, sizeof(acc_b));
}

void IMU_BMI088_FE_ProcessGyrRaw(IMU_BMI088_FE *fe,
                                 int16_t gx, int16_t gy, int16_t gz,
                                 float gyr_b_dps[3])
{
    if (!fe) return;

    /* raw -> 传感器坐标系物理量(dps) */
    float gyr_s_dps[3] = {
        (float)gx * fe->gyr_dps_per_lsb,
        (float)gy * fe->gyr_dps_per_lsb,
        (float)gz * fe->gyr_dps_per_lsb
    };

    /* 传感器系 -> 机体系 */
    float gyr_b[3];
    mat3x3_mul_vec_flat(fe->cfg.R_sb, gyr_s_dps, gyr_b);

    /* LPF（沿用现有 biquad 状态） */
    gyr_b[0] = biquad_proc(gyr_b[0], fe->lpf_g[0].b0, fe->lpf_g[0].b1, fe->lpf_g[0].b2,
                                      fe->lpf_g[0].a1, fe->lpf_g[0].a2, &fe->lpf_g[0].z1, &fe->lpf_g[0].z2);
    gyr_b[1] = biquad_proc(gyr_b[1], fe->lpf_g[1].b0, fe->lpf_g[1].b1, fe->lpf_g[1].b2,
                                      fe->lpf_g[1].a1, fe->lpf_g[1].a2, &fe->lpf_g[1].z1, &fe->lpf_g[1].z2);
    gyr_b[2] = biquad_proc(gyr_b[2], fe->lpf_g[2].b0, fe->lpf_g[2].b1, fe->lpf_g[2].b2,
                                      fe->lpf_g[2].a1, fe->lpf_g[2].a2, &fe->lpf_g[2].z1, &fe->lpf_g[2].z2);

    /* 温漂补偿：这里只用缓存的 last_temp_mC（不读温度） */
    const float T  = 0.001f * (float)fe->last_temp_mC;
    const float dT = T - fe->cfg.tc_T0;
    float tc[3] = {
        fe->cfg.tc_b0[0] + fe->cfg.tc_b1[0]*dT + fe->cfg.tc_b2[0]*dT*dT,
        fe->cfg.tc_b0[1] + fe->cfg.tc_b1[1]*dT + fe->cfg.tc_b2[1]*dT*dT,
        fe->cfg.tc_b0[2] + fe->cfg.tc_b1[2]*dT + fe->cfg.tc_b2[2]*dT*dT
    };

    /* 去 gyro bias（dps）+ 可选温补 */
    gyr_b[0] -= (fe->cfg.gyr_bias_dps[0] + (fe->cfg.tc_enable ? tc[0] : 0.0f));
    gyr_b[1] -= (fe->cfg.gyr_bias_dps[1] + (fe->cfg.tc_enable ? tc[1] : 0.0f));
    gyr_b[2] -= (fe->cfg.gyr_bias_dps[2] + (fe->cfg.tc_enable ? tc[2] : 0.0f));

    /* 输出 + 缓存 */
    if (gyr_b_dps) {
        gyr_b_dps[0] = gyr_b[0];
        gyr_b_dps[1] = gyr_b[1];
        gyr_b_dps[2] = gyr_b[2];
    }
    memcpy(fe->last_gyr_b_dps, gyr_b, sizeof(gyr_b));
}
