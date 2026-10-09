#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  FC_STEP_CONTINUE = 0,
  FC_STEP_STOP     = 1,
} fc_step_ret_t;

typedef enum {
    FC_MODE_ACRO  = 0,
    FC_MODE_ANGLE = 1,
} fc_mode_t;

#ifdef __cplusplus
}
#endif
