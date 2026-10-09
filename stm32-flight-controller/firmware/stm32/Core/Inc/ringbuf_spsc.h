#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef RB_SPSC_BARRIER
  #if defined(__ARM_ARCH_7EM__) || defined(__ARM_ARCH_7M__) || defined(__CORTEX_M)
    #if defined(__GNUC__) || defined(__clang__)
      #define RB_SPSC_BARRIER() __asm volatile ("dmb 0xF" ::: "memory")
    #else
      // Keil/ARMCLANG generally supports this intrinsic without needing core_cm4.h
      __asm void RB_SPSC_BARRIER_impl(void) { dmb; bx lr; }
      #define RB_SPSC_BARRIER() RB_SPSC_BARRIER_impl()
    #endif
  #else
    #define RB_SPSC_BARRIER() do{}while(0)
  #endif
#endif


typedef struct { uint32_t ts; float v[3]; } stamped_vec3_t;

/* Timestamp-only ring */
typedef struct {
    uint32_t *buf;
    uint16_t  mask;                 /* capacity-1 (capacity must be power-of-two) */
    volatile uint16_t head, tail;   /* head: producer; tail: consumer, plus IRQ-protected overflow drop */
} rb_ts_t;

/* Stamped vec3 ring */
typedef struct {
    stamped_vec3_t *buf;
    uint16_t  mask;
    volatile uint16_t head, tail;   /* head: producer; tail: consumer, plus IRQ-protected overflow drop */
} rb_vec_t;

/* ===== rb_ts_t ===== */
void rbts_init(rb_ts_t *r, uint32_t *buf, uint16_t cap_pow2);
static inline uint16_t rbts_capacity(const rb_ts_t *r) { return (uint16_t)(r->mask + 1u); }
uint16_t rbts_size(const rb_ts_t *r);
static inline int rbts_empty(const rb_ts_t *r) { return r->head == r->tail; }
int  rbts_push(rb_ts_t *r, uint32_t v);                  // return 1 if overwrote(drop oldest)
int rbts_pop(rb_ts_t *r, uint32_t *out);

/* ===== rb_vec_t ===== */
void rbv_init(rb_vec_t *r, stamped_vec3_t *buf, uint16_t cap_pow2);
static inline uint16_t rbv_capacity(const rb_vec_t *r) { return (uint16_t)(r->mask + 1u); }
uint16_t rbv_size(const rb_vec_t *r);
static inline int rbv_empty(const rb_vec_t *r) { return r->head == r->tail; }

int  rbv_push(rb_vec_t *r, uint32_t ts, const float v[3]); // return 1 if overwrote(drop oldest)
int  rbv_peek_oldest(const rb_vec_t *r, stamped_vec3_t *out, uint16_t *idx_out);
int  rbv_pop_oldest(rb_vec_t *r, stamped_vec3_t *out);
/* Access timestamp at logical ring index (index is in [tail, head)) */
uint32_t rbv_ts_at(const rb_vec_t *r, uint16_t i);
/* lower_bound(ts_target) on [tail, head): returns first i s.t. ts[i] >= target, or head */
uint16_t rbv_lower_bound_idx(const rb_vec_t *r, uint32_t ts_target);
/* Find index with timestamp nearest to (ts_g - dt_ag_ticks). Returns 1 if ok, 0 if empty. */
int  rbv_find_nearest_ts_corr(const rb_vec_t *r, uint32_t ts_g, int32_t dt_ag_ticks, uint16_t *idx_out);
/* Take element at logical index, dropping all older ones as well. */
stamped_vec3_t rbv_take_at(rb_vec_t *r, uint16_t idx);

/* Utility: wrap-safe absolute difference for uint32 */
static inline uint32_t rb_u32_absdiff(uint32_t a, uint32_t b) {
    return (uint32_t)((int32_t)(a - b) >= 0 ? (a - b) : (b - a));
}

#ifdef __cplusplus
}
#endif
