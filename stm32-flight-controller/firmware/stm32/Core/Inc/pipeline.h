#pragma once
#include <stdint.h>
#include "imu_bmi088_frontend.h"
#include "imu_bus_hal.h"   // 为了 IMU_BMI088_HALCtx, SPI_HandleTypeDef
#include "ringbuf_spsc.h"
#include "timebase.h"            // sec_to_ticks()
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 默认按你的策略：ACC 0.85*T，GYR 0.30*T（可被外部 #define 覆盖） */
#ifndef ACC_DEGLITCH_FRAC
#define ACC_DEGLITCH_FRAC 0.85f
#endif
#ifndef GYR_DEGLITCH_FRAC
#define GYR_DEGLITCH_FRAC 0.30f
#endif

#define IMU_DMA_MAX_XFER  8u   // ACC: 1(addr)+1(dummy)+6(data)=8; GYR: 1+6=7

typedef enum {
    PL_IF_NONE = 0,
    PL_IF_ACC  = 1,
    PL_IF_GYR  = 2,
} pl_inflight_t;

typedef struct {
    // IRQ 统计（窗口统计：reset_stats_window() 会清这些）
    volatile uint32_t acc_irq_cnt, gyr_irq_cnt;
    volatile uint32_t acc_dt_sum,  gyr_dt_sum;
    volatile uint32_t acc_dt_n,    gyr_dt_n;
    uint32_t min_acc_irq_dt, min_gyr_irq_dt;

    // ===== drop 统计（建议：累计值，不在 reset_stats_window() 清）=====
    volatile uint32_t acc_merge_drop;   // pending 合并/回填导致的丢（cnt-1 等）
    volatile uint32_t gyr_merge_drop;

    volatile uint32_t acc_sweep_drop;   // sweep_backlog() 主动丢弃
    volatile uint32_t gyr_sweep_drop;

    volatile uint32_t dma_timeout_cnt;  // cumulative DMA timeout recoveries
} IMU_PipelineStats;


/* 只绑定数据环（DMA pending -> data ring） */
void imu_pipeline_bind(rb_vec_t *acc_data, rb_vec_t *gyr_data);

/* 以“运行时采样率（Hz）”设置去抖阈值（ticks） */
void imu_pipeline_set_deglitch_by_fs(float acc_fs, float gyr_fs);
/* 也可直接用 ticks 设置（便于调试） */
void imu_pipeline_set_deglitch_ticks(uint32_t acc_ticks, uint32_t gyr_ticks);

/* —— ISR 转发目标：只需给时间戳 —— */
void imu_drdy_on_acc(uint32_t ts);
void imu_drdy_on_gyr(uint32_t ts);

/* —— 清道夫：控制积压上限 —— */
uint32_t imu_sweep_backlog(uint32_t *drop_acc, uint32_t *drop_gyr);

/* 取数/复位统计窗口（给 1s 打印用） */
const IMU_PipelineStats* imu_pipeline_stats(void);
void imu_pipeline_reset_stats_window(void);

/* 查询当前去抖阈值（ticks） */
uint32_t imu_pipeline_acc_deglitch_ticks(void);
uint32_t imu_pipeline_gyr_deglitch_ticks(void);

void imu_pipeline_dma_attach(IMU_BMI088_HALCtx *hal, IMU_BMI088_FE *fe);
void imu_pipeline_dma_poll(void);  // 主循环里兜底 kick 一下（可选但强烈建议）
void imu_pipeline_dma_on_spi_done(SPI_HandleTypeDef *hspi);
void imu_pipeline_dma_on_spi_error(SPI_HandleTypeDef *hspi);

#ifdef __cplusplus
}
#endif

