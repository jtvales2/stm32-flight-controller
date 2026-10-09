#include "fc_evt.h"
#include "timebase.h"
#include "stm32f4xx_hal.h"  // __DMB, __disable_irq, __get_PRIMASK

#ifndef FC_EVT_RB_SZ
#define FC_EVT_RB_SZ 64u   // must be power-of-two
#endif

#if ((FC_EVT_RB_SZ & (FC_EVT_RB_SZ - 1u)) != 0u)
#error FC_EVT_RB_SZ must be power-of-two
#endif

static fc_evt_t rb[FC_EVT_RB_SZ];
static volatile uint16_t head = 0; /* producer writes */
static volatile uint16_t tail = 0; /* consumer writes */
static volatile uint32_t drop_cnt = 0;

static inline uint32_t irq_save(void)
{
  uint32_t p = __get_PRIMASK();
  __disable_irq();
  return p;
}

static inline void irq_restore(uint32_t p)
{
  if (!p) __enable_irq();
}

void fc_evt_init(void)
{
  uint32_t p = irq_save();
  head = 0;
  tail = 0;
  drop_cnt = 0;
  irq_restore(p);
}

void fc_evt_push_isr(fc_evt_id_t id, int16_t a, int16_t b, int16_t c, int16_t d, uint32_t t)
{
  uint16_t h = head;
  uint16_t n = (uint16_t)((h + 1u) & (FC_EVT_RB_SZ - 1u));

  // Full -> drop newest (keep SPSC property: only consumer writes tail)
  if (n == tail) {
    drop_cnt++;
    return;
  }

  rb[h] = (fc_evt_t){ .t = t, .id = (uint16_t)id, .a = a, .b = b, .c = c, .d = d };
  __DMB();
  head = n;
}

void fc_evt_push(fc_evt_id_t id, int16_t a, int16_t b, int16_t c, int16_t d)
{
  fc_evt_push_isr(id, a, b, c, d, now_ticks());
}

int fc_evt_pop(fc_evt_t *out)
{
  uint16_t t = tail;
  if (t == head) return 0;

  __DMB();
  *out = rb[t];
  __DMB();
  tail = (uint16_t)((t + 1u) & (FC_EVT_RB_SZ - 1u));
  return 1;
}

uint32_t fc_evt_take_drop_count(void)
{
  uint32_t p = irq_save();
  uint32_t v = drop_cnt;
  drop_cnt = 0;
  irq_restore(p);
  return v;
}
