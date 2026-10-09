#ifndef FC_CORE_H
#define FC_CORE_H

#include <stdint.h>
#include "fc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void fc_core_step(const float gyr_dps[3], float dt_sec);
void fc_core_watchdog_1khz(void);
void fc_core_service_async(void);
void fc_core_log_poll(void);

void fc_core_baro_feed(uint8_t valid,
                       uint32_t ts_ticks,
                       int32_t press_pa,
                       int32_t temp_centi,
                       float alt_rel_m,
                       float vz_mps);

#ifdef __cplusplus
}
#endif

#endif

