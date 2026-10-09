#pragma once
#include <stdint.h>
#include <string.h>
#include "bmi08x.h"
#include "bmi088.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ====================== 运行时配置 ====================== */
typedef struct {
    /* 设备级 */
    uint8_t  use_sync;        /* 0=独立读, 1=开启 BMI088 accel-gyro 数据同步 */
    uint8_t  acc_range;       /* BMI088_ACCEL_RANGE_*   */
    uint8_t  gyr_range;       /* BMI08X_GYRO_RANGE_*    */
    uint8_t  acc_odr;         /* BMI08X_ACCEL_ODR_*     */
    uint8_t  gyr_odr;         /* BMI08X_GYRO_BW_*_ODR_* (枚举名里自带 ODR) */
    float    acc_lpf_cutoff_hz;  /* 二阶LPF 截止 */
    float    gyr_lpf_cutoff_hz;

    /* 前端修正 */
    float    R_sb[9];         /* 行主序: r00,r01,r02, r10,r11,r12, r20,r21,r22 */
    float    acc_bias_g[3];   /* 静态零偏(g) */
    float    gyr_bias_dps[3]; /* 静态零偏(dps) */

    /* 陀螺温漂补偿（可选）: bias_i(T) = b0 + b1*(T-T0) + b2*(T-T0)^2 */
    uint8_t  tc_enable;
    float    tc_T0;
    float    tc_b0[3], tc_b1[3], tc_b2[3];
} IMU_BMI088_Config;

/* ====================== 状态与自检 ====================== */
enum {
    IMU_BMI088_ST_OK                         = 0,
    IMU_BMI088_ST_APPLY_ERROR                = (1u<<0),
    IMU_BMI088_ST_ACC_ODR_MISMATCH           = (1u<<1),
    IMU_BMI088_ST_ACC_RANGE_MISMATCH         = (1u<<2),
    IMU_BMI088_ST_GYR_ODR_MISMATCH           = (1u<<3),
    IMU_BMI088_ST_GYR_RANGE_MISMATCH         = (1u<<4),
    IMU_BMI088_ST_SYNC_REQUESTED_BUT_UNAVAIL = (1u<<5),
};

typedef struct {
    int8_t     last_bosch_rslt; /* 最近一次 Bosch API 返回码 */
    uint32_t   flags;           /* 见上面的 bit 位 */
    float      runtime_acc_sample_hz;
    float      runtime_gyr_sample_hz;
} IMU_BMI088_Status;

/* ====================== 上下文 ====================== */
typedef struct {
    struct bmi08x_dev *dev;
    IMU_BMI088_Config cfg;
    IMU_BMI088_Status st;

    /* 换算（来自读回的 range） */
    float acc_g_per_lsb;
    float gyr_dps_per_lsb;

    /* LPF（二阶 biquad, 每轴一个）*/
    struct {
        float b0,b1,b2,a1,a2;
        float z1,z2;
    } lpf_a[3], lpf_g[3];

    /* 最近一次温度与输出 */
    int32_t last_temp_mC;
    float   last_acc_b_g[3];
    float   last_gyr_b_dps[3];
		
		/* ---- Sensor Time 基准 ---- */
		uint32_t st_last_24;   /* 上一帧 24-bit sensor time */
		uint8_t  st_has_last;  /* 是否已有上一帧 */
} IMU_BMI088_FE;

/* ====================== API ====================== */

/* 初始化 + 配置（会调用 bmi088_apply_config_file，必要时配置同步） */
int IMU_BMI088_FE_Init(IMU_BMI088_FE *fe, struct bmi08x_dev *dev, const IMU_BMI088_Config *cfg);

/* 运行时重配（ODR/量程/LPF/零偏/温补/安装矩阵皆可改），自动读回对拍并重算 biquad */
int IMU_BMI088_FE_Reconfigure(IMU_BMI088_FE *fe, const IMU_BMI088_Config *cfg);

/* 读取一帧，完成：raw->物理量->R_sb->LPF->温补->去偏，输出机体系（g/dps/°C）*/
int IMU_BMI088_FE_Read(IMU_BMI088_FE *fe, float acc_b_g[3], float gyr_b_dps[3], float *temp_C);

/* 手动设置零偏/温补/安装矩阵（无需重启） */
void IMU_BMI088_FE_SetBias(IMU_BMI088_FE *fe, const float acc_bias_g[3], const float gyr_bias_dps[3]);
void IMU_BMI088_FE_SetTempComp(IMU_BMI088_FE *fe, uint8_t enable, float T0,
                               const float b0[3], const float b1[3], const float b2[3]);
void IMU_BMI088_FE_SetMounting(IMU_BMI088_FE *fe, const float R_sb[9]);

/* 读回寄存器并校验与 cfg 是否一致；不一致置位 status.flags */
int IMU_BMI088_FE_AssertConfigConsistency(IMU_BMI088_FE *fe);

/* 获取状态结构体拷贝 */
static inline void IMU_BMI088_FE_GetStatus(const IMU_BMI088_FE *fe, IMU_BMI088_Status *out) {
    if (out) *out = fe->st;
}

/* 仅读取加速度（走同样的前端处理），输出机体系 g；可选回传温度 */
int IMU_BMI088_FE_ReadAccOnly(IMU_BMI088_FE *fe, float acc_b_g[3], float *temp_C);

/* 仅读取陀螺（走同样的前端处理：含温漂补偿），输出机体系 dps；可选回传温度 */
int IMU_BMI088_FE_ReadGyrOnly(IMU_BMI088_FE *fe, float gyr_b_dps[3], float *temp_C);


/* API：读取一帧并输出 dt（秒），dt 基于 accel sensor_time */
int IMU_BMI088_FE_ReadTimed(IMU_BMI088_FE *fe,
                            float acc_b_g[3], float gyr_b_dps[3], float *temp_C,
                            float *dt_sec);

// 自动找平：采样 N 帧平均重力，左乘到 R_sb（只校 R/P）
int IMU_BMI088_FE_AutoLevel(IMU_BMI088_FE *fe, uint16_t samples, uint16_t inter_ms);

void IMU_BMI088_FE_ProcessAccRaw(IMU_BMI088_FE *fe, int16_t ax, int16_t ay, int16_t az, float acc_b_g[3]);
void IMU_BMI088_FE_ProcessGyrRaw(IMU_BMI088_FE *fe, int16_t gx, int16_t gy, int16_t gz, float gyr_b_dps[3]);

#ifdef __cplusplus
}
#endif
