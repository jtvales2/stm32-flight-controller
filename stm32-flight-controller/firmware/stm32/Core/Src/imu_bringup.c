#include "bmi08x.h"
#include "bmi088.h"
#include "imu_bmi088_bus.h"
#include "imu_bus_hal.h"
#include "imu_bmi088_frontend.h"
#include "spi.h"
#include "gpio.h"
#include "imu_bringup.h"
#include "stm32f4xx.h"
#include "log_uart.h"   // 新增

#ifndef IMU_BRINGUP_LOG_EN
#define IMU_BRINGUP_LOG_EN  1   // 0=彻底静音；1=只打印错误
#endif

#if IMU_BRINGUP_LOG_EN
  #define IMU_EPRINTF(...)  log_uart_printf(__VA_ARGS__)
#else
  #define IMU_EPRINTF(...)  do{}while(0)
#endif

static IMU_BMI088_FE *s_fe = NULL;
		
/* 关同步 + 配 DRDY 成双通道中断 */
static int imu_enable_dual_drdy(IMU_BMI088_FE *fe)
{
    struct bmi08x_accel_int_channel_cfg a = {
        .int_channel = BMI08X_INT_CHANNEL_1,
        .int_type    = BMI08X_ACCEL_DATA_RDY_INT,
        .int_pin_cfg = {
            .lvl          = BMI08X_INT_ACTIVE_HIGH,
            .output_mode  = BMI08X_INT_MODE_PUSH_PULL,
            .enable_int_pin = BMI08X_ENABLE,
        }
    };

    if (bmi08a_set_int_config(&a, fe->dev) != BMI08X_OK)
        return -1;

    struct bmi08x_gyro_int_channel_cfg g = {
        .int_channel = BMI08X_INT_CHANNEL_3,
        .int_type    = BMI08X_GYRO_DATA_RDY_INT,
        .int_pin_cfg = {
            .lvl          = BMI08X_INT_ACTIVE_HIGH,
            .output_mode  = BMI08X_INT_MODE_PUSH_PULL,
            .enable_int_pin = BMI08X_ENABLE,
        }
    };

    return (bmi08g_set_int_config(&g, fe->dev) == BMI08X_OK) ? 0 : -2;
}

/* 初始化 IMU（SPI1，双 DRDY，不启用同步）*/
int imu_bringup_init(IMU_BMI088_Bus *bus,
                     IMU_BMI088_FE  *fe)
{
    static IMU_BMI088_HALCtx halctx;
	
	  s_fe = NULL;   // 防止半初始化状态被外部误用
	
    /* 绑定 SPI1 + 两个 CS 引脚 */
    imu_bmi088_hal_init_ctx(&halctx, &hspi1,
                            GPIOC, CS1_Pin,   /* ACC CS */
                            GPIOC, CS2_Pin);  /* GYR CS */
    halctx.spi_timeout_ms = 1000;             /* 可选：不设就默认 1000 */

    int rs = imu_bmi088_bus_open_hal(bus, &halctx);
    if (rs) {
        IMU_EPRINTF("bus open fail %d\r\n", rs);
        return rs;
    }

    IMU_BMI088_Config cfg = {
        .use_sync = 0,                                    // 双 DRDY
        .acc_range = BMI088_ACCEL_RANGE_6G,
        .gyr_range = BMI08X_GYRO_RANGE_1000_DPS,
        .acc_odr   = BMI08X_ACCEL_ODR_800_HZ,            // A 800 Hz
        .gyr_odr   = BMI08X_GYRO_BW_116_ODR_1000_HZ,     // G 1000 Hz
        .acc_lpf_cutoff_hz = 120.0f,                     //统一 LPF 截止
        .gyr_lpf_cutoff_hz = 120.0f,                     //统一 LPF 截止
        .tc_enable = 0,
        .tc_T0     = 30.0f,
    };

    rs = IMU_BMI088_FE_Init(fe, imu_bmi088_bus_dev(bus), &cfg);
    if (rs) {
        IMU_EPRINTF("fe init fail %d\r\n", rs);
        return rs;
    }

    /* 安装矩阵：按你板子实际方向改；此处示例为单位阵（X前 Y右 Z下 已在前端定义一致） */
    const float R9[9] = { 1,0,0,  0,-1,0,  0,0,-1 };
    IMU_BMI088_FE_SetMounting(fe, R9);
    IMU_BMI088_FE_AssertConfigConsistency(fe);
		
		s_fe = fe;
		
    /* 配置 ACC/GRY 双 DRDY 中断 */
    rs = imu_enable_dual_drdy(fe);
    if (rs) {
       IMU_EPRINTF("dual-DRDY cfg fail %d\r\n", rs);
        return rs;
    }

    return 0;
}

int imu_get_gyro_bias_dps(float out_dps[3])
{
    if (!s_fe || !out_dps) return -1;
    out_dps[0] = s_fe->cfg.gyr_bias_dps[0];
    out_dps[1] = s_fe->cfg.gyr_bias_dps[1];
    out_dps[2] = s_fe->cfg.gyr_bias_dps[2];
    return 0;
}

int imu_set_gyro_bias_dps(const float b_dps[3])
{
    if (!s_fe || !b_dps) return -1;

    float acc_b[3] = {
        s_fe->cfg.acc_bias_g[0],
        s_fe->cfg.acc_bias_g[1],
        s_fe->cfg.acc_bias_g[2],
    };
    float gyr_b[3] = { b_dps[0], b_dps[1], b_dps[2] };

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    IMU_BMI088_FE_SetBias(s_fe, acc_b, gyr_b);
    __set_PRIMASK(primask);

    return 0;
}
