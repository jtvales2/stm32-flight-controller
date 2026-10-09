#ifndef FC_CONTROL_H
#define FC_CONTROL_H

#include <stdint.h>
#include "fc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

fc_step_ret_t fc_ctrl_run(const float gyr_dps[3], float dt, uint8_t skip_id);

#ifdef __cplusplus
}
#endif

#endif
