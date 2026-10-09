#ifndef FC_YAW_H
#define FC_YAW_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float yaw_out;
    float yaw_gain;
    float thr_used;
    uint8_t air_eff;
} fc_yaw_out_t;

void fc_yaw_update(float yaw_err,
                   float rc_yaw,
                   float thr_base,
                   float idle_now,
                   float dt,
                   uint8_t skip_id,
                   fc_yaw_out_t *out);

void fc_yaw_debug_poll(const fc_yaw_out_t *yaw_out, float rc_yaw);

#ifdef __cplusplus
}
#endif

#endif
