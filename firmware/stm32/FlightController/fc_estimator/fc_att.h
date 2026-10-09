#pragma once
#include <stdint.h>
#include "fc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

fc_step_ret_t fc_att_update(void);                 // ∂¡ Mahony -> s.att.raw_*
fc_step_ret_t fc_att_apply_offsets_and_check(void);// raw -> use + isfinite ºÏ≤È
uint8_t       fc_is_level_ok(void);                // π© arming / reset service ”√

#ifdef __cplusplus
}
#endif
