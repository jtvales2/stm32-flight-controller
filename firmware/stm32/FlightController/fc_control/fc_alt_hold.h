#ifndef FC_ALT_HOLD_H
#define FC_ALT_HOLD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float thr_target;
    uint8_t active;
} fc_alt_hold_out_t;

uint8_t fc_alt_hold_should_run(void);

void fc_alt_hold_update(float manual_thr,
                        float dt,
                        uint8_t skip_id,
                        fc_alt_hold_out_t *out);

#ifdef __cplusplus
}
#endif

#endif
