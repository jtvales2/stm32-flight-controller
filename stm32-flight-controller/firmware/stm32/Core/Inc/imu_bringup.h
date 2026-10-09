#pragma once
#include "imu_bmi088_frontend.h"
#include "imu_bmi088_bus.h"
#ifdef __cplusplus
extern "C" {
#endif

int  imu_set_gyro_bias_dps(const float b_dps[3]);
int  imu_get_gyro_bias_dps(float out_dps[3]);

/* 初始化 BMI088：打开 SPI1，总线 + 前端 + 双 DRDY + AutoLevel */
int imu_bringup_init(IMU_BMI088_Bus *bus,
                     IMU_BMI088_FE  *fe);

#ifdef __cplusplus
}
#endif
