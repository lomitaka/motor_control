#pragma once
#include <stdint.h>

typedef struct DebugInfo {
    /** Current step interval, encoded by the positioning-stepper driver. */
    volatile int64_t current_interval;
    /** Target step interval, encoded by the positioning-stepper driver. */
    volatile int64_t target_interval;
    /** Remaining interval until the next scheduled step event. */
    volatile int64_t remainining_interval;
    /** Estimated number of steps required to brake to a stop. */
    volatile uint16_t braking_distance;
} DebugInfo;