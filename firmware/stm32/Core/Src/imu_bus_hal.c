#include "imu_bus_hal.h"

/* 直接把 user 视作 HALCtx 使用 */
typedef IMU_BMI088_HALCtx _ctx_t;

/* 片选：which=ACC/GYR, level=0/1 */
static void _hal_cs(void *user, int which, int level)
{
  _ctx_t *u = (_ctx_t*)user;
  GPIO_PinState s = level ? GPIO_PIN_SET : GPIO_PIN_RESET;
  if (which == IMU_BMI088_CS_ACC) {
		HAL_GPIO_WritePin(u->acc_cs_port, u->acc_cs_pin, s);
  } else {
		HAL_GPIO_WritePin(u->gyr_cs_port, u->gyr_cs_pin, s);
  }
}

/* 发送 */
static int _hal_tx(void *user, const uint8_t *tx, uint16_t len)
{
  _ctx_t *u = (_ctx_t*)user;
  return (HAL_SPI_Transmit(u->hspi, (uint8_t*)tx, len, u->spi_timeout_ms) == HAL_OK) ? 0 : -1;
}

/* 接收 */
static int _hal_rx(void *user, uint8_t *rx, uint16_t len)
{
	_ctx_t *u = (_ctx_t*)user;
  return (HAL_SPI_Receive(u->hspi, rx, len, u->spi_timeout_ms) == HAL_OK) ? 0 : -1;
}

/* 毫秒延时 */
static void _hal_delay(uint32_t ms)
{
  HAL_Delay(ms);
}

/* HAL 版 BusOps */
static const IMU_BMI088_BusOps _hal_ops = {
    .cs       = _hal_cs,
    .tx       = _hal_tx,
    .rx       = _hal_rx,
    .delay_ms = _hal_delay,
};

/* 一把梭：绑定 HAL ops+ctx → 调通用 open */
int imu_bmi088_bus_open_hal(IMU_BMI088_Bus *bus, IMU_BMI088_HALCtx *ctx)
{
    if (!ctx) return BMI08X_E_NULL_PTR;;
    if (ctx->spi_timeout_ms == 0) ctx->spi_timeout_ms = 1000u;
    return imu_bmi088_bus_open(bus, &_hal_ops, (void*)ctx);
}
