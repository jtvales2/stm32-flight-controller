#ifndef FC_RATE_H
#define FC_RATE_H

#include <stdint.h>
#include "fc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float roll_out;
    float pitch_out;

    float yaw_err;
    float yaw_rate_meas;
} fc_rate_out_t;

fc_step_ret_t fc_rate_update_roll_pitch(const float gyr_dps[3],
                                        float roll_rate_cmd,
                                        float pitch_rate_cmd,
                                        float yaw_rate_cmd,
                                        float dt,
                                        uint8_t skip_id,
                                        fc_rate_out_t *out);

#ifdef __cplusplus
}
#endif

#endif
