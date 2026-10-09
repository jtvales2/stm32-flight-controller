#include "log_uart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ===== 可配置 ===== */
#ifndef LOG_RB_CAP
#define LOG_RB_CAP        2048u      /* 必须 2^N：1024/2048/4096... */
#endif

#ifndef LOG_TX_CHUNK
#define LOG_TX_CHUNK      128u       /* 每次发多少字节 */
#endif

#ifndef LOG_PRINTF_TMP
#define LOG_PRINTF_TMP    320u       /* ISR里vsnprintf临时buf，别太大 */
#endif

#ifndef LOG_UART_USE_DMA
#define LOG_UART_USE_DMA  1          /* 你的 USART1 已配 DMA TX，建议=1 */
#endif

#if (LOG_RB_CAP < 2u)
#error "LOG_RB_CAP must be >= 2"
#endif

#if ((LOG_RB_CAP & (LOG_RB_CAP - 1u)) != 0u)
#error "LOG_RB_CAP must be power-of-two"
#endif

typedef struct {
  uint8_t  buf[LOG_RB_CAP];
  volatile uint32_t head;      /* producer 写 */
  volatile uint32_t tail;      /* consumer 释放（在 TxCplt 推进） */
  volatile uint32_t drop_cnt;  /* 溢出丢字节 */
} log_rb_t;

static UART_HandleTypeDef *s_huart = NULL;
static log_rb_t s_rb;

static volatile uint8_t  s_tx_busy = 0;
static uint16_t          s_inflight = 0;
static uint8_t           s_txbuf[LOG_TX_CHUNK];

static inline uint32_t rb_mask(void) { return (LOG_RB_CAP - 1u); }

uint32_t log_uart_drop_cnt(void) { return s_rb.drop_cnt; }

void log_uart_init(UART_HandleTypeDef *huart)
{
  s_huart = huart;
  s_rb.head = 0;
  s_rb.tail = 0;
  s_rb.drop_cnt = 0;
  s_tx_busy = 0;
  s_inflight = 0;
}

/* 控制环/ISR写入：单生产者（控制环） */
static void rb_write_bytes_isr(const uint8_t *data, uint32_t len)
{
  if (!data || len == 0) return;

  uint32_t head = s_rb.head;
  uint32_t tail = s_rb.tail;           // 可能略旧，但只会更保守
  uint32_t used = head - tail;
  uint32_t free = LOG_RB_CAP - used;

  if (free == 0u) {
    s_rb.drop_cnt += len;
    return;
  }

  uint32_t n = (len <= free) ? len : free;
  uint32_t m = rb_mask();

  // head 当前落点
  uint32_t idx = head & m;
  // 从 idx 到 buf 末尾最多能连续写多少
  uint32_t first = LOG_RB_CAP - idx;
  if (first > n) first = n;

  memcpy(&s_rb.buf[idx], data, first);

  uint32_t rest = n - first;
  if (rest) {
    memcpy(&s_rb.buf[0], data + first, rest);
  }

  __DMB();               // 先保证数据可见，再发布 head
  s_rb.head = head + n;

  if (n < len) {
    s_rb.drop_cnt += (len - n);
  }
}


int log_uart_printf(const char *fmt, ...)
{
  if (!s_huart || !fmt) return 0;

  char tmp[LOG_PRINTF_TMP];

  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
  va_end(ap);

  if (n <= 0) return n;

  /* 截断也无所谓：vsnprintf保证\0 */
  uint32_t wlen = (n < (int)sizeof(tmp)) ? (uint32_t)n : (uint32_t)(sizeof(tmp) - 1u);
  rb_write_bytes_isr((const uint8_t *)tmp, wlen);
  return n;
}

void log_uart_poll(void)
{
  if (!s_huart) return;
  if (s_tx_busy) return;

  uint32_t head = s_rb.head;
  uint32_t tail = s_rb.tail;
  uint32_t avail = head - tail;
  if (avail == 0u) return;

  uint32_t n = (avail < LOG_TX_CHUNK) ? avail : LOG_TX_CHUNK;
  uint32_t m = rb_mask();

  uint32_t idx = tail & m;
  uint32_t first = LOG_RB_CAP - idx;
  if (first > n) first = n;

  memcpy(&s_txbuf[0], &s_rb.buf[idx], first);

  uint32_t rest = n - first;
  if (rest) {
    memcpy(&s_txbuf[first], &s_rb.buf[0], rest);
  }

  HAL_StatusTypeDef st;
#if LOG_UART_USE_DMA
  st = HAL_UART_Transmit_DMA(s_huart, s_txbuf, (uint16_t)n);
#else
  st = HAL_UART_Transmit_IT(s_huart, s_txbuf, (uint16_t)n);
#endif

  if (st == HAL_OK) {
    s_inflight = (uint16_t)n;
    s_tx_busy  = 1;
  }
  /* HAL_BUSY/ERROR：不推进tail，下一次poll再试 */
}

void log_uart_on_tx_cplt(UART_HandleTypeDef *huart)
{
  if (!s_huart) return;
  if (huart != s_huart) return;

  uint32_t tail = s_rb.tail;
  tail += (uint32_t)s_inflight;

  __DMB();
  s_rb.tail = tail;

  s_inflight = 0;
  s_tx_busy = 0;
}
