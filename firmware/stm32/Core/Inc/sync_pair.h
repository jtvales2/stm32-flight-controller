#pragma once
#include <stdint.h>
#include "ringbuf_spsc.h"
#include "imu_bmi088_frontend.h"
#include "fusion_mahony.h"

#ifdef __cplusplus
extern "C" {
#endif

	
typedef struct { 
	uint32_t last_acc_age_ticks; // accel 相对 gyro 的滞后（ticks）
} IMU_SyncPairStats;

typedef enum {
    IMU_SP_OK         =  1,   // 成功消费 1 个 gyro 样本，并已尝试配对 accel + 更新 Mahony
    IMU_SP_NO_DATA    =  0,   // gyr_rb 为空，本次没干活

    IMU_SP_E_BADPTR   = -1,   // 传入指针为空 / 未 bind
    IMU_SP_E_TS_ORDER = -2,   // gyro 时间戳倒退/重复（乱序）
} IMU_SyncPairRet;

/**
 * 消费 1 个 gyro 样本（gyro-driven 的核心步进）
 *
 * @param gyr_dps_out  输出：本帧 gyro (dps)，可为 NULL（不需要就传 NULL）
 * @param dt_sec_out   输出：本帧 dt (sec)，可为 NULL
 *
 * @return
 *   IMU_SP_OK(1)      : 成功消费 1 帧 gyro（已更新 Mahony）
 *   IMU_SP_NO_DATA(0) : gyro ring 为空，没做任何事
 *   <0                : 错误（例如时间戳倒退/未初始化）
 */
int imu_sync_pair_step_one(float gyr_dps_out[3], float *dt_sec_out);

/* 兼容旧字段名（过渡期用，之后可以删） */
#define last_pair_resid_ticks last_acc_age_ticks

/* 绑定依赖：谁提供数据、谁吃掉融合结果 */
void imu_sync_pair_bind(IMU_BMI088_FE *fe,
                        Mahony *mah,
                        rb_vec_t *acc_rb,
                        rb_vec_t *gyr_rb);

/* 告诉它运行时采样率（Hz），用于算配对窗口 */
void imu_sync_pair_set_fs(float acc_fs, float gyr_fs);

void imu_sync_pair_reset(void);
/* 每次最多融合 Nmax 对（就是现在的 fuse_some_pairs 的职能） */
int imu_sync_pair_fuse_some(int Nmax);

/* 给 main 打印用的统计接口 */
const IMU_SyncPairStats* imu_sync_pair_stats(void);
// TEMP(imu): 过渡接口，为了让飞控层先跑起来。
// TODO(cleanup): 等 imu_api.h/imu_state.c 建好后，把这些 getter 移过去并删除此处。
// Delete-when: FC 改为只依赖 imu_api.h
/* 当前欧拉角（rad），机体系：x前 y右 z下，对应 roll/pitch/yaw */
void imu_get_attitude_euler(float *roll, float *pitch, float *yaw);

/* 当前陀螺输出（dps），机体系 p=x, q=y, r=z */
void imu_get_gyro_dps(float *gx, float *gy, float *gz);

uint32_t imu_get_gyro_seq(void);
uint32_t imu_get_gyro_ts(void);

#ifdef __cplusplus
}
#endif
