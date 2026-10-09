#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 模式：6DoF(无磁) / 9DoF(含磁) */
typedef enum {
    MAHONY_MODE_6DOF = 0,
    MAHONY_MODE_9DOF = 1
} MahonyMode;

/* 运行时配置（可随时改） */
typedef struct {
    MahonyMode mode;

    /* 增益：把加计/磁计误差拆开更好调 */
    float kp_acc;    /* 加计比例增益（姿态收敛速度） */
    float kp_mag;    /* 磁计比例增益（航向收敛速度） */
    float ki;        /* 陀螺零偏积分增益 */

    /* 传感器质量门限 + 灰度权重（连续权重而非开关） */
    /* 加计：以 1g 为参考，落入 [g_min,g_max] 认为“可信”，否则按灰度衰减 */
    float acc_g_min;       /* 典型 0.7 */
    float acc_g_max;       /* 典型 1.3 */
    float acc_gray_k;      /* 超界后权重衰减斜率，0~几，典型 2.0 */

    /* 磁计：以当地磁场强度 (uT) 为参考 */
    float mag_ref_uT;      /* 典型 45uT，可在线估计 */
    float mag_uT_min;      /* 典型 20uT */
    float mag_uT_max;      /* 典型 70uT */
    float mag_gray_k;      /* 权重衰减斜率，典型 1.5 */

    /* 数值安全：dt 下界/上界（秒），防止积分爆炸 */
    float dt_min, dt_max;
} MahonyConfig;

/* 运行状态可见：用于调参与监控 */
typedef struct {
    float q[4];            /* 当前姿态四元数 (w,x,y,z) */
    float gyro_bias[3];    /* 积分估计的零偏(dps) */

    /* 质量信息/最近一帧的权重和范数 */
    uint8_t acc_used;      /* 这帧加计是否参与（>0） */
    uint8_t mag_used;      /* 这帧磁计是否参与（>0） */
    float   acc_norm_g;    /* |a| in g */
    float   mag_norm_uT;   /* |m| in uT */
    float   w_acc;         /* 加计灰度权重 0~1 */
    float   w_mag;         /* 磁计灰度权重 0~1 */

    float   last_dt;       /* 上一帧 dt */
} MahonyStatus;

/* 融合器上下文 */
typedef struct {
    MahonyConfig cfg;
    MahonyStatus st;
} Mahony;

/* === API === */

/* 初始化：q0 可为 NULL→置单位四元数 */
void mahony_init(Mahony* f, const MahonyConfig* cfg, const float q0[4]);

/* 在线重配：所有字段可改（不会重置姿态/偏置） */
void mahony_reconfigure(Mahony* f, const MahonyConfig* cfg);

/* 重置姿态/偏置（保留 cfg） */
void mahony_reset(Mahony* f, const float q0[4]);

/* 6DoF 更新（单位：g / dps）*/
void mahony_update_6dof(Mahony* f,
                         float gx_dps, float gy_dps, float gz_dps,
                         float ax_g,   float ay_g,   float az_g,
                         float dt);

/* 9DoF 更新（单位：g / dps / uT）*/
void mahony_update_9dof(Mahony* f,
                         float gx_dps, float gy_dps, float gz_dps,
                         float ax_g,   float ay_g,   float az_g,
                         float mx_uT,  float my_uT,  float mz_uT,
                         float dt);

/* 便捷：把当前四元数转欧拉（rad） */
void mahony_get_euler(const Mahony* f, float* roll, float* pitch, float* yaw);

/* 便捷：设置/读取姿态 */
static inline void mahony_get_q(const Mahony* f, float q[4]) { q[0]=f->st.q[0]; q[1]=f->st.q[1]; q[2]=f->st.q[2]; q[3]=f->st.q[3]; }
static inline void mahony_set_q(Mahony* f, const float q[4]) { f->st.q[0]=q[0]; f->st.q[1]=q[1]; f->st.q[2]=q[2]; f->st.q[3]=q[3]; }

#ifdef __cplusplus
}
#endif
