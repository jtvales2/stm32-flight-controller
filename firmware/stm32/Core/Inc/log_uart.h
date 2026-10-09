#pragma once

#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void log_uart_init(UART_HandleTypeDef *huart);

/* 控制环/ISR：只入队，不发串口、不阻塞 */
int  log_uart_printf(const char *fmt, ...);

/* 主循环/低优先级：触发一次异步发送（DMA 或 IT） */
void log_uart_poll(void);

/* HAL_UART_TxCpltCallback 里调用：清 busy + 推进 tail */
void log_uart_on_tx_cplt(UART_HandleTypeDef *huart);

/* 可选：溢出丢字节计数 */
uint32_t log_uart_drop_cnt(void);

#ifdef __cplusplus
}
#endif
