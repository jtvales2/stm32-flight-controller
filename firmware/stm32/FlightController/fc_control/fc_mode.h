#ifndef FC_MODE_H
#define FC_MODE_H

#include <stdint.h>
#include "fc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void fc_mode_update(void);
fc_mode_t fc_mode_get(void);
const char *fc_mode_str(fc_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif
