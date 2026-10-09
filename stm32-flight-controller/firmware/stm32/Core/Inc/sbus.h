#pragma once
#include <stdint.h>

typedef struct {
    uint16_t ch[16];      // 0..15
    uint8_t  lost_frame;  // flag bit
    uint8_t  failsafe;    // flag bit
    uint32_t frame_cnt;   // 收到的有效帧计数
} sbus_frame_t;

void sbus_feed_bytes(const uint8_t *data, uint16_t len);
int  sbus_read_latest(sbus_frame_t *out); // 有新帧返回1
uint16_t sbus_to_us(uint16_t v); // 172..1811 → 1000..2000us
