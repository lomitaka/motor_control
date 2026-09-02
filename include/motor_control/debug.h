#pragma once
#include <stdint.h>

typedef struct DebugInfo {
    volatile uint16_t tcnt1_a;
    volatile uint16_t tcnt1_b;
    volatile int64_t current_interval;
    volatile int64_t previous_interval;
    volatile int64_t target_interval;
    volatile int64_t remainining_interval;
    volatile uint16_t braking_distance;
    volatile uint16_t update_no;
    volatile uint8_t mask;
} DebugInfo;