#pragma once
#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef struct {
    uint8_t  valid;        // 1=输出可用（P0建立完成）
    uint32_t ts_ticks;     // now_ticks() 时间戳（us ticks）
    int32_t  press_pa;     // 压力 Pa
    int32_t  temp_centi;   // 温度 0.01C
    float    alt_rel_m;    // 相对高度 m
    float    vz_mps;       // 爬升率 m/s（滤波后）
} fc_baro_out_t;

// 绑定 SPI + CS，初始化 MS56xx（不做阻塞延时）
HAL_StatusTypeDef fc_baro_init_spi(SPI_HandleTypeDef *hspi,
                                   GPIO_TypeDef *cs_port, uint16_t cs_pin);

// 重置内部状态（让它重新建 P0）
void fc_baro_reset(void);

// Use the latest good pressure as ground reference and clear alt/vz filters.
// Returns 1 on success, 0 if the barometer is not ready yet.
uint8_t fc_baro_zero_to_current(void);

// 非阻塞调度入口：内部限频（默认2ms）
// 返回 1：本次有新样本并更新 out
// 返回 0：无新样本/未到时间/未初始化
uint8_t fc_baro_service_sched(fc_baro_out_t *out);

