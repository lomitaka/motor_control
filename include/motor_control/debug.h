#pragma once
#include <stdint.h>

typedef struct DebugInfo {
    int64_t current_interval;
    int64_t target_interval;
    int64_t remainining_interval;
    uint8_t curr_rate;
    uint8_t tar_rate;
} DebugInfo;