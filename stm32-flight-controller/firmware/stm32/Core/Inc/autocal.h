#pragma once
#include <stdint.h>
#include "ringbuf_spsc.h"
#include "imu_bmi088_frontend.h"
#include "fusion_mahony.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化：告诉模块用哪个 FE / Mahony */
void imu_autocal_init(IMU_BMI088_FE *fe, Mahony *mah);
void imu_autocal_set_enabled(uint8_t enabled);

/* 每对已经配好的 A / G 调用一次 */
void maybe_static_calibrate_gz(const stamped_vec3_t *A,
                               const stamped_vec3_t *G);

#ifdef __cplusplus
}
#endif
