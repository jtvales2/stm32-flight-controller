#ifndef FC_THROTTLE_H
#define FC_THROTTLE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void fc_ctrl_idle_ramp_update(float dt);
void fc_ctrl_thr_update(float dt, uint8_t skip_id);

#ifdef __cplusplus
}
#endif

#endif
