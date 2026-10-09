#pragma once
#include <stdint.h>

static inline int16_t q100(float x)  { return (int16_t)(x * 100.0f); }
static inline int16_t q10(float x)   { return (int16_t)(x * 10.0f);  }
static inline int16_t q1000(float x) { return (int16_t)(x * 1000.0f); }
