#pragma once
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "imu_bmi088_bus.h"   // 只用到 IMU_BMI088_Bus / IMU_BMI088_BusOps

#ifdef __cplusplus
extern "C" {
#endif

/* HAL 侧上下文：SPI + 两路片选 + 超时 */
typedef struct {
	SPI_HandleTypeDef *hspi;
  GPIO_TypeDef *acc_cs_port; uint16_t acc_cs_pin;  /* ACC CS */
  GPIO_TypeDef *gyr_cs_port; uint16_t gyr_cs_pin;  /* GYR CS */
  uint32_t spi_timeout_ms;                         /* 默认 1000 ms */
} IMU_BMI088_HALCtx;

static inline void imu_bmi088_hal_init_ctx(IMU_BMI088_HALCtx *ctx,
                                           SPI_HandleTypeDef *hspi,
                                           GPIO_TypeDef *acc_cs_port, uint16_t acc_cs_pin,
                                           GPIO_TypeDef *gyr_cs_port, uint16_t gyr_cs_pin)
{
  ctx->hspi = hspi;
  ctx->acc_cs_port = acc_cs_port; ctx->acc_cs_pin = acc_cs_pin;
  ctx->gyr_cs_port = gyr_cs_port; ctx->gyr_cs_pin = gyr_cs_pin;
  ctx->spi_timeout_ms = 1000u;
}

/* 工厂式 open：把 HAL 胶水“绑定”为 BusOps+user，再调用通用 open */
int imu_bmi088_bus_open_hal(IMU_BMI088_Bus *bus, IMU_BMI088_HALCtx *ctx);

#ifdef __cplusplus
}
#endif
