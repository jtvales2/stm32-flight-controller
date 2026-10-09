#include "sbus.h"
#include "stm32f4xx.h"
#include <string.h>

#define SBUS_START 0x0F
#define SBUS_LEN   25

static uint8_t  fr[SBUS_LEN];
static uint8_t  idx = 0;
static uint8_t  collecting = 0;

static sbus_frame_t last;
static volatile int new_frame = 0;

// bit解包：sbus[1..22]是16个通道的11位数据
static void sbus_unpack(const uint8_t *b, sbus_frame_t *o) {
    const uint8_t *s = b; // b[0]=0x0F, b[1..22]=data, b[23]=flags, b[24]=end(0x00)
    o->ch[ 0] = ((s[1]     | s[2]<<8)                & 0x07FF);
    o->ch[ 1] = (((s[2]>>3)| s[3]<<5)                & 0x07FF);
    o->ch[ 2] = (((s[3]>>6)| s[4]<<2 | s[5]<<10)     & 0x07FF);
    o->ch[ 3] = (((s[5]>>1)| s[6]<<7)                & 0x07FF);
    o->ch[ 4] = (((s[6]>>4)| s[7]<<4)                & 0x07FF);
    o->ch[ 5] = (((s[7]>>7)| s[8]<<1 | s[9]<<9)      & 0x07FF);
    o->ch[ 6] = (((s[9]>>2)| s[10]<<6)               & 0x07FF);
    o->ch[ 7] = (((s[10]>>5)| s[11]<<3)              & 0x07FF);
    o->ch[ 8] = ((s[12]    | s[13]<<8)               & 0x07FF);
    o->ch[ 9] = (((s[13]>>3)| s[14]<<5)              & 0x07FF);
    o->ch[10] = (((s[14]>>6)| s[15]<<2 | s[16]<<10)  & 0x07FF);
    o->ch[11] = (((s[16]>>1)| s[17]<<7)              & 0x07FF);
    o->ch[12] = (((s[17]>>4)| s[18]<<4)              & 0x07FF);
    o->ch[13] = (((s[18]>>7)| s[19]<<1 | s[20]<<9)   & 0x07FF);
    o->ch[14] = (((s[20]>>2)| s[21]<<6)              & 0x07FF);
    o->ch[15] = (((s[21]>>5)| s[22]<<3)              & 0x07FF);

    uint8_t flags = s[23];
    o->lost_frame =  (flags & 0x04) ? 1 : 0;
    o->failsafe   =  (flags & 0x08) ? 1 : 0;
}

// 供USART6 的 IDLE 回调反复调用
void sbus_feed_bytes(const uint8_t *data, uint16_t len) {
    for (uint16_t i = 0; i < len; ++i) {
        uint8_t c = data[i];
        if (!collecting) {
            if (c == SBUS_START) { fr[0] = c; idx = 1; collecting = 1; }
            continue;
        }
        fr[idx++] = c;
        if (idx == SBUS_LEN) {
            // 结束校验：尾字节常见为0x00（部分接收机也可能是0x04）
            if (fr[24] == 0x00 || fr[24] == 0x04) {
                sbus_unpack(fr, &last);
                last.frame_cnt++;
                new_frame = 1;
            }
            collecting = 0;
            idx = 0;
        }
    }
}

int sbus_read_latest(sbus_frame_t *out) {
    int available = 0;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (new_frame) {
        *out = last;
        new_frame = 0;
        available = 1;
    }
    __set_PRIMASK(primask);
    return available;
}

uint16_t sbus_to_us(uint16_t v) {
    if (v < 172)  v = 172;
    if (v > 1811) v = 1811;
    // 172..1811 → 1000..2000
    return 1000 + (uint32_t)(v - 172) * 1000 / (1811 - 172);
}
