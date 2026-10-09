#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float throttle;   // 0.0 ~ 1.0
    float roll;       // -1.0 ~ 1.0
    float pitch;      // -1.0 ~ 1.0
    float yaw;        // -1.0 ~ 1.0
} mixer_cmd_t;

/* QuadX mixer：输出原始混控结果（可能超出 0~1），限幅在上层做 */
void mixer_quadx(const mixer_cmd_t *u, float out[4]);

#ifdef __cplusplus
}
#endif
