#pragma once
#include <stdint.h>

typedef struct DebugInfo {
    int64_t current_interval;
    int64_t target_interval;
    int64_t remainining_interval;
} DebugInfo;