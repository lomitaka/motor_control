#pragma once
#include <stdint.h>

typedef struct DebugInfo {
    uint64_t current_interval;
    uint64_t target_interval;
    uint64_t remainining_interval;
} DebugInfo;