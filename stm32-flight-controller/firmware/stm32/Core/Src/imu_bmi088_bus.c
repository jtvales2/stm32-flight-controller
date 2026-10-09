#include "imu_bmi088_bus.h"
#include <string.h>
#include <stdio.h>

#ifndef IMU_BMI088_DEBUG
#define IMU_BMI088_DEBUG 0
#endif
#if IMU_BMI088_DEBUG
  #define IMU_BMI_LOG(...)  printf(__VA_ARGS__)
#else
  #define IMU_BMI_LOG(...)  ((void)0)
#endif

/* ---- 私有：全局指针让 Bosch 回调能拿到 bus/ops （BMI08x 没有 context 指针） ---- */
static IMU_BMI088_Bus *s_bus = NULL;

/* 读取：CS低 -> 发reg -> (Bosch会把len加上dummy) -> 连续接收len字节 -> CS高 */
static int8_t s_bmi_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	if (!s_bus || !s_bus->ops || !s_bus->ops->cs || !s_bus->ops->tx || !s_bus->ops->rx)
		return BMI08X_E_NULL_PTR;

  if (dev_id != s_bus->dev.accel_id && dev_id != s_bus->dev.gyro_id)
		return BMI08X_E_DEV_NOT_FOUND;

  const int which = (dev_id == s_bus->dev.accel_id) ? IMU_BMI088_CS_ACC : IMU_BMI088_CS_GYR;

  s_bus->ops->cs(s_bus->user, which, 0);                 // CS LOW

  if (s_bus->ops->tx(s_bus->user, &reg_addr, 1) != 0) {  // 先发寄存器地址（上层已带R/W位）
		s_bus->ops->cs(s_bus->user, which, 1);
    return BMI08X_E_COM_FAIL;
  }

  // 直接按 len 接收（如果是 ACC，上层已把 len 加上了 dummy；我们不丢弃）
  uint16_t remaining = len;
  while (remaining) {
		uint16_t n = (remaining > IMU_BMI088_MAX_RW) ? IMU_BMI088_MAX_RW : remaining;
    if (s_bus->ops->rx(s_bus->user, data, n) != 0) {
			s_bus->ops->cs(s_bus->user, which, 1);
      return BMI08X_E_COM_FAIL;
		}
		data      += n;
		remaining -= n;
	}
  s_bus->ops->cs(s_bus->user, which, 1);                 // CS HIGH
  return BMI08X_OK;
}


/* 写入：CS低 -> 发reg -> 分块发送data -> CS高 */
static int8_t s_bmi_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    // 同理：先判空
    if (!s_bus || !s_bus->ops || !s_bus->ops->cs || !s_bus->ops->tx)
        return BMI08X_E_NULL_PTR;

    // 再判 dev_id
    if (dev_id != s_bus->dev.accel_id && dev_id != s_bus->dev.gyro_id)
        return BMI08X_E_DEV_NOT_FOUND;

    const int which = (dev_id == s_bus->dev.accel_id) ? IMU_BMI088_CS_ACC : IMU_BMI088_CS_GYR;

    s_bus->ops->cs(s_bus->user, which, 0);                    // CS LOW

    if (s_bus->ops->tx(s_bus->user, &reg_addr, 1) != 0) {
        s_bus->ops->cs(s_bus->user, which, 1);                // CS HIGH
        return BMI08X_E_COM_FAIL;
    }

    uint16_t remaining = len;
    while (remaining) {
        const uint16_t n = (remaining > IMU_BMI088_MAX_RW) ? IMU_BMI088_MAX_RW : remaining;
        if (s_bus->ops->tx(s_bus->user, data, n) != 0) {
            s_bus->ops->cs(s_bus->user, which, 1);            // CS HIGH
            return BMI08X_E_COM_FAIL;
        }
        data += n;
        remaining -= n;
    }

    s_bus->ops->cs(s_bus->user, which, 1);                    // CS HIGH
    return BMI08X_OK;
}


/* ---- 公共 API ---- */
int imu_bmi088_bus_open(IMU_BMI088_Bus *bus, const IMU_BMI088_BusOps *ops, void *user)
{
    if (!bus || !ops || !ops->cs || !ops->tx || !ops->rx || !ops->delay_ms)
        return BMI08X_E_NULL_PTR;

    memset(bus, 0, sizeof(*bus));
    bus->ops  = ops;
    bus->user = user;
    s_bus     = bus;  // 回调需要

    struct bmi08x_dev *dev = &bus->dev;
    dev->intf           = BMI08X_SPI_INTF;
    dev->accel_id       = IMU_BMI088_CS_ACC;
    dev->gyro_id        = IMU_BMI088_CS_GYR;
    dev->read           = s_bmi_spi_read;
    dev->write          = s_bmi_spi_write;
    dev->delay_ms       = ops->delay_ms;
    dev->read_write_len = 32;   // 配置流要求
    dev->dummy_byte     = 1;    // ACC SPI 读需要丢 1 个 dummy

    // CS 拉高稳态
    ops->cs(user, IMU_BMI088_CS_ACC, 1);
    ops->cs(user, IMU_BMI088_CS_GYR, 1);
    ops->delay_ms(5);

    /* === 新增：热启动友好，先软复位 A/G === */
    (void)bmi08a_soft_reset(dev);
    dev->delay_ms(5);
    (void)bmi08g_soft_reset(dev);
    dev->delay_ms(30);
		
    // 初始化
    int8_t rs = bmi088_init(dev);
    if (rs != BMI08X_OK) return rs;
		
		/* === 新增：确保 ACC 不在省电/挂起态，再配流（兼容当前 Bosch 驱动） === */
    /* 1) 关闭 Advanced Power Save：ACC_PWR_CONF(0x7C) bit0 = 0 */
    uint8_t reg = 0;
    if (bmi08a_get_regs(BMI08X_ACCEL_PWR_CONF_REG, &reg, 1, dev) == BMI08X_OK) {  // 寄存器地址宏见头文件
        reg &= (uint8_t)~0x01;                       // bit0=adv_power_save
        (void)bmi08a_set_regs(BMI08X_ACCEL_PWR_CONF_REG, &reg, 1, dev);
        dev->delay_ms(BMI08X_POWER_CONFIG_DELAY);
    }

    /* 2) 置加速度计为 ACTIVE：这版 API 通过 dev->accel_cfg.power 传参 */
    dev->accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
    (void)bmi08a_set_power_mode(dev);                // 正确签名：只收 dev*
    dev->delay_ms(BMI08X_POWER_CONFIG_DELAY);
		
		/* === Ensure GYRO is in NORMAL power mode (not SUSPEND) === */
		dev->gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;
		rs = bmi08g_set_power_mode(dev);
		if (rs != BMI08X_OK) return rs;
		dev->delay_ms(BMI08X_POWER_CONFIG_DELAY);
		
    // 配置流（一次重试，仍失败直接返回）
    dev->delay_ms(5);
    rs = bmi088_apply_config_file(dev);
    if (rs == BMI08X_E_CONFIG_STREAM_ERROR) {
        dev->delay_ms(10);
        rs = bmi088_apply_config_file(dev);
    }
    if (rs != BMI08X_OK) return rs;

		// config file 后，再确保 ACC 退出 advanced power save
		uint8_t pwr_conf = 0;
		if (bmi08a_get_regs(BMI08X_ACCEL_PWR_CONF_REG, &pwr_conf, 1, dev) == BMI08X_OK) {
			pwr_conf &= (uint8_t)~0x01; // bit0=adv_power_save -> 0
      (void)bmi08a_set_regs(BMI08X_ACCEL_PWR_CONF_REG, &pwr_conf, 1, dev);
      dev->delay_ms(BMI08X_POWER_CONFIG_DELAY);
		}

    bus->opened = 1;
    return BMI08X_OK;
}


int imu_bmi088_bus_reset(IMU_BMI088_Bus *bus)
{
    if (!bus) return BMI08X_E_NULL_PTR;
    struct bmi08x_dev *dev = &bus->dev;

    (void)bmi08a_soft_reset(dev);
    (void)bmi08g_soft_reset(dev);
    dev->delay_ms(10);

    int8_t rs = bmi088_init(dev);
    if (rs != BMI08X_OK) return rs;

    dev->delay_ms(5);
    rs = bmi088_apply_config_file(dev);
    return rs;
}


void imu_bmi088_bus_close(IMU_BMI088_Bus *bus)
{
    if (!bus) return;
    if (s_bus == bus) s_bus = NULL;
    bus->opened = 0;
}

int imu_bmi088_bus_read_reg(IMU_BMI088_Bus *bus, int which, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (!bus || !bus->opened) return BMI08X_E_NULL_PTR;
    uint8_t id = (which == IMU_BMI088_CS_ACC) ? bus->dev.accel_id : bus->dev.gyro_id;
    return s_bmi_spi_read(id, reg, buf, len);
}

int imu_bmi088_bus_write_reg(IMU_BMI088_Bus *bus, int which, uint8_t reg, const uint8_t *data, uint16_t len)
{
    if (!bus || !bus->opened) return BMI08X_E_NULL_PTR;
    /* Bosch 回调签名要求非 const 指针，这里安全地去 const */
    uint8_t *p = (uint8_t*)data;
    uint8_t id = (which == IMU_BMI088_CS_ACC) ? bus->dev.accel_id : bus->dev.gyro_id;
    return s_bmi_spi_write(id, reg, p, len);
}
