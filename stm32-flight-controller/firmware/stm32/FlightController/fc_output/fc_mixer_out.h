#ifndef FC_MIXER_OUT_H
#define FC_MIXER_OUT_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} fc_mixer_cmd_t;

void fc_mixer_output(const fc_mixer_cmd_t *u,
                     float out_min,
                     float out_max);

#ifdef __cplusplus
}
#endif

#endif
