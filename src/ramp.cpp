#include <math.h>
#include "motor_control/ramp.h"


    // current value moves to target; returns new current value
    float Ramp::step(RampType type, float current, float target, float time_ms, float dt) {
        if (type == RampType::None || time_ms <= 0.0f) return target;
        float diff = target - current;
        float stepMax = fabs(diff) * (dt / time_ms); // linear proportion of remaining gap
        if (type == RampType::Linear) {
            // Linear: move proportionally to dt/rampTime (clamped)
            if (fabs(diff) <= stepMax) return target;
            return current + (diff > 0 ? stepMax : -stepMax);
        } else { // SCurve: ease in/out (simple cubic ease)
            // map progress = dt / rampTime as incremental easing — simple and cheap
            // better approach: keep elapsed/time state; here simple approximation
            float sign = (diff >= 0) ? 1.0f : -1.0f;
            float mag = fabs(diff);
            // cubic easing step ~ (dt/rampTime)^3 * mag but scale to be useful:
            float factor = dt / time_ms; // 0..1
            if (factor > 1.0f) factor = 1.0f;
            float inc = mag * (factor * (3 - 2 * factor)); // smoothstep
            if (inc > mag) inc = mag;
            return current + sign * inc;
        }
    }