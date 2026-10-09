#include "fusion_mahony.h"
#include <math.h>
#include <string.h>

/* ---------- 内部小工具 ---------- */
static inline float fast_inv_sqrtf(float x) {
    /* 纯 C 的 1/sqrt(x)；对 STM32F4 已足够 */
    return 1.0f / sqrtf(x);
}

static inline void q_normalize(float q[4]) {
    float n = q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3];
    if (n > 0.0f) {
        float inv = 1.0f / sqrtf(n);
        q[0]*=inv; q[1]*=inv; q[2]*=inv; q[3]*=inv;
    } else {
        q[0]=1; q[1]=q[2]=q[3]=0;
    }
}

/* 由四元数估计重力方向（机体系） */
static inline void q_estimate_gravity(const float q[4], float v[3]) {
    float q0=q[0], q1=q[1], q2=q[2], q3=q[3];
    v[0] = 2.0f*(q1*q3 - q0*q2);
    v[1] = 2.0f*(q0*q1 + q2*q3);
    v[2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;
}

/* Mahony 原法：由四元数+磁场估计参考磁场方向 bx/bz ，再得到机体系期望磁向 */
static inline void q_reference_mag(const float q[4], const float m_body[3],
                                   float *bx, float *bz, float v_m[3])
{
    /* 把机体系磁测量旋到地理坐标系：h = q ? m ? q* */
    float q0=q[0], q1=q[1], q2=q[2], q3=q[3];
    float q0q0=q0*q0, q0q1=q0*q1, q0q2=q0*q2, q0q3=q0*q3;
    float q1q1=q1*q1, q1q2=q1*q2, q1q3=q1*q3;
    float q2q2=q2*q2, q2q3=q2*q3;
    float q3q3=q3*q3;

    float hx = 2.0f*(m_body[0]*(0.5f - q2q2 - q3q3) + m_body[1]*(q1q2 - q0q3) + m_body[2]*(q1q3 + q0q2));
    float hy = 2.0f*(m_body[0]*(q1q2 + q0q3) + m_body[1]*(0.5f - q1q1 - q3q3) + m_body[2]*(q2q3 - q0q1));
    float hz = 2.0f*(m_body[0]*(q1q3 - q0q2) + m_body[1]*(q2q3 + q0q1) + m_body[2]*(0.5f - q1q1 - q2q2));

    *bx = sqrtf(hx*hx + hy*hy);
    *bz = hz;

    /* 再把参考磁向（bx,0,bz）旋回机体系，得到期望机体系磁向 v_m */
    /* v_m = R(q) * [bx,0,bz] */
    float vx = 2.0f*( (0.5f - q2q2 - q3q3)*(*bx) + (q1q3 + q0q2)*(*bz) );
    float vy = 2.0f*( (q1q2 + q0q3)*(*bx) + (q2q3 - q0q1)*(*bz) );
    float vz = 2.0f*( (q1q3 - q0q2)*(*bx) + (0.5f - q1q1 - q2q2)*(*bz) );
    v_m[0]=vx; v_m[1]=vy; v_m[2]=vz;

    /* 只关方向，单位化 */
    float inv = fast_inv_sqrtf(v_m[0]*v_m[0]+v_m[1]*v_m[1]+v_m[2]*v_m[2]);
    v_m[0]*=inv; v_m[1]*=inv; v_m[2]*=inv;
}

/* 低成本 clamp */
static inline float clamp01(float x){ return (x<0.f)?0.f:((x>1.f)?1.f:x); }

/* 根据范数做“灰度权重”：落在 [lo,hi]→1，之外线性衰减到 0（斜率 k） */
static inline float gray_weight(float norm, float lo, float hi, float k, float ref)
{
    if (norm <= 0.f) return 0.f;
    if (norm >= lo && norm <= hi) return 1.f;
    float err = (norm - ref);
    float w = 1.f - k * fabsf(err) / (hi - lo + 1e-6f);
    return clamp01(w);
}

/* ---------- 公共 API ---------- */

void mahony_init(Mahony* f, const MahonyConfig* cfg, const float q0[4])
{
    memset(f, 0, sizeof(*f));
    f->cfg = *cfg;
    f->st.q[0] = q0 ? q0[0] : 1.f;
    f->st.q[1] = q0 ? q0[1] : 0.f;
    f->st.q[2] = q0 ? q0[2] : 0.f;
    f->st.q[3] = q0 ? q0[3] : 0.f;
    q_normalize(f->st.q);

    /* 默认 dt 边界，避免除零/爆 dt */
    if (f->cfg.dt_min <= 0.f) f->cfg.dt_min = 1e-4f;
    if (f->cfg.dt_max <= f->cfg.dt_min) f->cfg.dt_max = 0.05f; /* 20Hz 下界 */
    if (f->cfg.mag_ref_uT <= 0.f) f->cfg.mag_ref_uT = 45.f;
    if (f->cfg.acc_g_min <= 0.f) { f->cfg.acc_g_min = 0.7f; f->cfg.acc_g_max = 1.3f; }
    if (f->cfg.mag_uT_min <= 0.f) { f->cfg.mag_uT_min = 20.f; f->cfg.mag_uT_max = 70.f; }
}

void mahony_reconfigure(Mahony* f, const MahonyConfig* cfg)
{
    f->cfg = *cfg;
}

void mahony_reset(Mahony* f, const float q0[4])
{
    f->st.q[0] = q0 ? q0[0] : 1.f;
    f->st.q[1] = q0 ? q0[1] : 0.f;
    f->st.q[2] = q0 ? q0[2] : 0.f;
    f->st.q[3] = q0 ? q0[3] : 0.f;
    f->st.gyro_bias[0]=f->st.gyro_bias[1]=f->st.gyro_bias[2]=0.f;
    q_normalize(f->st.q);
}

static void step_common(Mahony* f,
                        float gx_dps, float gy_dps, float gz_dps,
                        float ax_g,   float ay_g,   float az_g,
                        const float* m_uT, /* 可为 NULL */
                        float dt)
{
    /* dt 裁剪 */
    if (dt < f->cfg.dt_min) dt = f->cfg.dt_min;
    if (dt > f->cfg.dt_max) dt = f->cfg.dt_max;
    f->st.last_dt = dt;

    /* 陀螺：dps→rad/s，并减去估计偏置 */
    const float d2r = 0.017453292519943295f;
    float gx = (gx_dps - f->st.gyro_bias[0]) * d2r;
    float gy = (gy_dps - f->st.gyro_bias[1]) * d2r;
    float gz = (gz_dps - f->st.gyro_bias[2]) * d2r;

    /* 归一化加计方向、计算权重 */
    float anorm = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
    f->st.acc_norm_g = anorm;
    float ax=0, ay=0, az=0;
    if (anorm > 1e-6f) { ax=ax_g/anorm; ay=ay_g/anorm; az=az_g/anorm; }
    float w_acc = gray_weight(anorm, f->cfg.acc_g_min, f->cfg.acc_g_max,
                              f->cfg.acc_gray_k, 1.0f);
    f->st.w_acc = w_acc;
    f->st.acc_used = (w_acc > 0.f);

    /* 估计当前重力方向 */
    float v_g[3]; q_estimate_gravity(f->st.q, v_g);

    /* 误差：测量方向 × 估计方向（叉乘） */
    float ex = 0.f, ey = 0.f, ez = 0.f;
    if (f->st.acc_used) {
        ex += (ay*v_g[2] - az*v_g[1]);
        ey += (az*v_g[0] - ax*v_g[2]);
        ez += (ax*v_g[1] - ay*v_g[0]);
        ex *= f->cfg.kp_acc * w_acc;
        ey *= f->cfg.kp_acc * w_acc;
        ez *= f->cfg.kp_acc * w_acc;
    }

    /* 磁计（可选） */
    if (m_uT) {
        float mx_uT=m_uT[0], my_uT=m_uT[1], mz_uT=m_uT[2];
        float mnorm = sqrtf(mx_uT*mx_uT + my_uT*my_uT + mz_uT*mz_uT);
        f->st.mag_norm_uT = mnorm;
        float wx = gray_weight(mnorm, f->cfg.mag_uT_min, f->cfg.mag_uT_max,
                               f->cfg.mag_gray_k, f->cfg.mag_ref_uT);
        f->st.w_mag = wx;
        f->st.mag_used = (wx > 0.f && f->cfg.mode == MAHONY_MODE_9DOF);

        if (f->st.mag_used) {
            /* 单位化的机体系磁向 */
            float invm = (mnorm > 1e-6f) ? (1.0f/mnorm) : 0.f;
            float mb[3] = { mx_uT*invm, my_uT*invm, mz_uT*invm };

            /* 计算参考磁向（Mahony 的 bx/bz），得到期望机体系磁向 v_m */
            float bx, bz, v_m[3];
            q_reference_mag(f->st.q, mb, &bx, &bz, v_m);

            /* 误差累加（磁向叉乘） */
            float exm = (mb[1]*v_m[2] - mb[2]*v_m[1]);
            float eym = (mb[2]*v_m[0] - mb[0]*v_m[2]);
            float ezm = (mb[0]*v_m[1] - mb[1]*v_m[0]);
            ex += f->cfg.kp_mag * wx * exm;
            ey += f->cfg.kp_mag * wx * eym;
            ez += f->cfg.kp_mag * wx * ezm;
        }
    } else {
        f->st.mag_used = 0; f->st.w_mag = 0.f; f->st.mag_norm_uT = 0.f;
    }

    /* 积分校正零偏 */
    if (f->cfg.ki > 0.f) {
        f->st.gyro_bias[0] += f->cfg.ki * ex * dt * 57.2957795f; /* 回到 dps 单位 */
        f->st.gyro_bias[1] += f->cfg.ki * ey * dt * 57.2957795f;
        f->st.gyro_bias[2] += f->cfg.ki * ez * dt * 57.2957795f;
    }

    /* 比例校正到陀螺（rad/s） */
    gx += ex;
    gy += ey;
    gz += ez;

    /* 四元数积分 dq = 0.5*q?ω*dt */
    float q0=f->st.q[0], q1=f->st.q[1], q2=f->st.q[2], q3=f->st.q[3];
    float halfdt = 0.5f*dt;
    f->st.q[0] += (-q1*gx - q2*gy - q3*gz) * halfdt;
    f->st.q[1] += ( q0*gx + q2*gz - q3*gy) * halfdt;
    f->st.q[2] += ( q0*gy - q1*gz + q3*gx) * halfdt;
    f->st.q[3] += ( q0*gz + q1*gy - q2*gx) * halfdt;

    q_normalize(f->st.q);
}

void mahony_update_6dof(Mahony* f,
                        float gx_dps, float gy_dps, float gz_dps,
                        float ax_g,   float ay_g,   float az_g,
                        float dt)
{
    f->cfg.mode = MAHONY_MODE_6DOF;
    step_common(f, gx_dps,gy_dps,gz_dps, ax_g,ay_g,az_g, NULL, dt);
}

void mahony_update_9dof(Mahony* f,
                        float gx_dps, float gy_dps, float gz_dps,
                        float ax_g,   float ay_g,   float az_g,
                        float mx_uT,  float my_uT,  float mz_uT,
                        float dt)
{
    f->cfg.mode = MAHONY_MODE_9DOF;
    float m[3] = {mx_uT, my_uT, mz_uT};
    step_common(f, gx_dps,gy_dps,gz_dps, ax_g,ay_g,az_g, m, dt);
}

/* 欧拉角（Z-Y-X，航向-俯仰-横滚） */
void mahony_get_euler(const Mahony* f, float* roll, float* pitch, float* yaw)
{
    const float q0=f->st.q[0], q1=f->st.q[1], q2=f->st.q[2], q3=f->st.q[3];
    if (roll)  *roll  = atan2f(2.f*(q0*q1+q2*q3), 1.f - 2.f*(q1*q1 + q2*q2));
    if (pitch) *pitch = asinf( 2.f*(q0*q2 - q3*q1) );
    if (yaw)   *yaw   = atan2f(2.f*(q0*q3+q1*q2), 1.f - 2.f*(q2*q2 + q3*q3));
}
