#pragma once
#include <stdint.h>

typedef struct DebugInfo {
    volatile int64_t current_interval;
    volatile int64_t target_interval;
    volatile int64_t remainining_interval;
    volatile uint16_t braking_distance;
} DebugInfo;