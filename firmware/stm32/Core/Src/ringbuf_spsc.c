#include "ringbuf_spsc.h"
#include "stm32f4xx_hal.h"

/* ===== internals ===== */
static inline uint16_t rb_span(uint16_t head, uint16_t tail) {
    return (uint16_t)(head - tail); /* modulo-2^16 arithmetic */
}

/*
 * The normal SPSC ownership is producer=head, consumer=tail.
 * This implementation also supports "drop oldest on full", which means the
 * producer may advance tail during overflow. Any code path that can update
 * tail is therefore IRQ-protected to avoid ISR/main read-modify-write races.
 */
static inline uint32_t rb_irq_save(void) {
#if defined(__ARM_ARCH_7EM__) || defined(__ARM_ARCH_7M__) || defined(__CORTEX_M)
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return primask;
#else
    return 0u;
#endif
}

static inline void rb_irq_restore(uint32_t primask) {
#if defined(__ARM_ARCH_7EM__) || defined(__ARM_ARCH_7M__) || defined(__CORTEX_M)
    __DMB();
    __set_PRIMASK(primask);
#else
    (void)primask;
#endif
}

/* ===== rb_ts_t ===== */
void rbts_init(rb_ts_t *r, uint32_t *buf, uint16_t cap_pow2) {
    r->buf = buf;
    r->mask = (uint16_t)(cap_pow2 - 1u);
    r->head = r->tail = 0;
}

uint16_t rbts_size(const rb_ts_t *r) {
    return rb_span(r->head, r->tail);
}

int rbts_push(rb_ts_t *r, uint32_t v) {
    uint16_t h_next = (uint16_t)(r->head + 1u);
    int overwrote = 0;

    if (rb_span(h_next, r->tail) > r->mask) {
        uint32_t primask = rb_irq_save();
        if (rb_span(h_next, r->tail) > r->mask) {
            r->tail++;
            overwrote = 1;
        }
        rb_irq_restore(primask);
    }

    r->buf[r->head & r->mask] = v;
    RB_SPSC_BARRIER();
    r->head = h_next;
    return overwrote;
}

int rbts_pop(rb_ts_t *r, uint32_t *out) {
    uint32_t primask;

    primask = rb_irq_save();
    if (r->tail == r->head) {
        rb_irq_restore(primask);
        return 0;
    }

    RB_SPSC_BARRIER(); /* acquire before reading data */
    *out = r->buf[r->tail & r->mask];
    r->tail++;
    rb_irq_restore(primask);
    return 1;
}

/* ===== rb_vec_t ===== */
void rbv_init(rb_vec_t *r, stamped_vec3_t *buf, uint16_t cap_pow2) {
    r->buf = buf; r->mask = (uint16_t)(cap_pow2 - 1u); r->head = r->tail = 0;
}

uint16_t rbv_size(const rb_vec_t *r) {
    return rb_span(r->head, r->tail);
}

int rbv_push(rb_vec_t *r, uint32_t ts, const float v[3]) {
    uint16_t h_next = (uint16_t)(r->head + 1u);
    int overwrote = 0;

    if (rb_span(h_next, r->tail) > r->mask) {
        uint32_t primask = rb_irq_save();
        if (rb_span(h_next, r->tail) > r->mask) {
            r->tail++;              /* drop oldest */
            overwrote = 1;
        }
        rb_irq_restore(primask);
    }

    stamped_vec3_t s = { ts, { v[0], v[1], v[2] } };
    r->buf[r->head & r->mask] = s;
    RB_SPSC_BARRIER();
    r->head = h_next;
    return overwrote;
}

int rbv_peek_oldest(const rb_vec_t *r, stamped_vec3_t *out, uint16_t *idx_out) {
    uint32_t primask;

    primask = rb_irq_save();
    if (r->tail == r->head) {
        rb_irq_restore(primask);
        return 0;
    }

    RB_SPSC_BARRIER(); /* acquire */
    if (idx_out) *idx_out = r->tail;
    *out = r->buf[r->tail & r->mask];
    rb_irq_restore(primask);
    return 1;
}

int rbv_pop_oldest(rb_vec_t *r, stamped_vec3_t *out) {
    uint32_t primask;

    primask = rb_irq_save();
    if (r->tail == r->head) {
        rb_irq_restore(primask);
        return 0;
    }

    RB_SPSC_BARRIER(); /* acquire */
    *out = r->buf[r->tail & r->mask];
    r->tail++;
    rb_irq_restore(primask);
    return 1;
}

uint32_t rbv_ts_at(const rb_vec_t *r, uint16_t i) {
    RB_SPSC_BARRIER();
    return r->buf[i & r->mask].ts;
}

uint16_t rbv_lower_bound_idx(const rb_vec_t *r, uint32_t ts_target) {
    uint16_t lo = r->tail, hi = r->head;
    while (lo < hi) {
        uint16_t mid = (uint16_t)(lo + ((hi - lo) >> 1));
        if (rbv_ts_at(r, mid) < ts_target) lo = (uint16_t)(mid + 1);
        else hi = mid;
    }
    return lo; /* possibly == head */
}

int rbv_find_nearest_ts_corr(const rb_vec_t *r, uint32_t ts_g, int32_t dt_ag_ticks, uint16_t *idx_out) {
    if (r->tail == r->head) return 0;
    uint32_t target = (uint32_t)((int64_t)ts_g - (int64_t)dt_ag_ticks);
    uint16_t i = rbv_lower_bound_idx(r, target);
    uint16_t best = (i == r->tail) ? i :
                    (i == r->head) ? (uint16_t)(i - 1) :
                    (rb_u32_absdiff(target, rbv_ts_at(r,(uint16_t)(i-1))) <= rb_u32_absdiff(target, rbv_ts_at(r,i)) ? (uint16_t)(i-1) : i);
    *idx_out = best;
    return 1;
}

stamped_vec3_t rbv_take_at(rb_vec_t *r, uint16_t idx) {
    uint32_t primask;
    stamped_vec3_t val;

    primask = rb_irq_save();
    RB_SPSC_BARRIER(); /* acquire before reading chosen element */
    val = r->buf[idx & r->mask];

    /* drop older */
    while (r->tail != idx) r->tail++;

    RB_SPSC_BARRIER();
    r->tail++; /* drop the chosen one */
    rb_irq_restore(primask);
    return val;
}
