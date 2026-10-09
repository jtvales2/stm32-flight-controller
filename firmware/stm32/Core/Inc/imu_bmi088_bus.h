#pragma once
#include <stdint.h>
#include <stddef.h>
#include "bmi08x.h"
#include "bmi088.h"
#ifdef __cplusplus
extern "C" {
#endif

/* 设备选择：加速度计/陀螺的“虚拟地址”。用于区分两条 CS 线 */
enum {
  IMU_BMI088_CS_ACC = 0,
  IMU_BMI088_CS_GYR = 1
};

/* 可调：单次最大收发长度（Bosch 默认 32，写配置流也按该长度分块） */
#ifndef IMU_BMI088_MAX_RW
#define IMU_BMI088_MAX_RW 32
#endif

typedef void (*imu_bmi088_cs_ctrl_t)(void *user, int which, int level); // 0=LOW, 1=HIGH
typedef int  (*imu_bmi088_tx_t)(void *user, const uint8_t *tx, uint16_t len);
typedef int  (*imu_bmi088_rx_t)(void *user,       uint8_t *rx, uint16_t len);
typedef void (*imu_bmi088_delay_ms_t)(uint32_t ms);

typedef struct {
	imu_bmi088_cs_ctrl_t cs;   // 片选
  imu_bmi088_tx_t      tx;   // 只发（HAL_SPI_Transmit）
  imu_bmi088_rx_t      rx;   // 只收（HAL_SPI_Receive）
  imu_bmi088_delay_ms_t delay_ms;
} IMU_BMI088_BusOps;


/* bus 对象（仅保存 Bosch dev 和回调） */
typedef struct {
  struct bmi08x_dev dev;          /* 直接暴露给前端层使用 */
  const IMU_BMI088_BusOps *ops;   /* 平台回调 */
  void *user;                     /* 平台回调的 user 数据 */
  int opened;                     /* 状态标记 */
} IMU_BMI088_Bus;

/* === API === */

/* 打开 bus：绑定 SPI/延时回调 → bmi088_init → bmi088_apply_config_file
 * 成功返回 0，失败返回负值（Bosch 定义的错误码）
 */
int imu_bmi088_bus_open(IMU_BMI088_Bus *bus, const IMU_BMI088_BusOps *ops, void *user);

/* 软复位（acc/gyr）+ 重新上传配置流。成功返回 0。 */
int imu_bmi088_bus_reset(IMU_BMI088_Bus *bus);

/* 关闭 bus（标记关闭；不动硬件） */
void imu_bmi088_bus_close(IMU_BMI088_Bus *bus);

/* 低级寄存器访问（直通 Bosch 的 read/write 回调），便于自检/对拍 */
int imu_bmi088_bus_read_reg(IMU_BMI088_Bus *bus, int which, uint8_t reg, uint8_t *buf, uint16_t len);
int imu_bmi088_bus_write_reg(IMU_BMI088_Bus *bus, int which, uint8_t reg, const uint8_t *data, uint16_t len);

/* 拿到底层 Bosch dev 指针（给前端层做 ODR/量程/中断等配置） */
static inline struct bmi08x_dev* imu_bmi088_bus_dev(IMU_BMI088_Bus *bus) { return &bus->dev; }


/* （此处不再暴露任何 HAL 相关定义；保持 bus 层纯净） */
#ifdef __cplusplus
}
#endif
