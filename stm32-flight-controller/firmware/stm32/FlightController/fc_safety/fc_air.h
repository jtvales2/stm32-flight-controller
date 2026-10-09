#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void fc_air_reset(void);
void fc_air_update(const float gyr_dps[3]);

#ifdef __cplusplus
}
#endif
