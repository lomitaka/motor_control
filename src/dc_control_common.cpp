#ifdef SIMULATION_MODE
#include "../simulator/avr_mock.h"
#else
#include <avr/io.h>
#include <avr/interrupt.h>
#endif

#include "internals/arduino.h"
#include "internals/dc_scheduler.h"
#include "internals/fces.h"

namespace {

constexpr uint8_t MAX_MOTOR_COUNT = 5;
constexpr uint16_t TIMER_GUARD_TICKS = 100;
constexpr uint16_t PWM_PERIOD_TICKS = 64000;

// One slot represents one PWM output. The same slot can also describe an H-bridge.
struct MotorSlot {
    volatile bool active;
    volatile bool hbridge;
    volatile int8_t pwm_pin;
    volatile uint8_t dir_a_pin;
    volatile uint8_t dir_b_pin;
    volatile int16_t target;
    volatile int16_t current;
    volatile uint8_t acceleration_step;
};

volatile MotorSlot motors[MAX_MOTOR_COUNT] = {
    {false, false, -1, 0, 0, 0, 0, 1},
    {false, false, -1, 0, 0, 0, 0, 1},
    {false, false, -1, 0, 0, 0, 0, 1},
    {false, false, -1, 0, 0, 0, 0, 1},
    {false, false, -1, 0, 0, 0, 0, 1}
};

// PWM channels are switched off in ascending duty-cycle order.
volatile uint8_t off_order[MAX_MOTOR_COUNT] = {0, 0, 0, 0, 0};
volatile uint8_t off_order_count = 0;
volatile uint8_t current_off_order_index = 0;

void writeDirection(uint8_t motor_index, int16_t value) {
    if (!motors[motor_index].hbridge) {
        return;
    }

    // Both direction pins LOW means the bridge is disabled while stopped.
    if (value > 0) {
        motor_control_internals::digitalWrite(motors[motor_index].dir_a_pin, HIGH);
        motor_control_internals::digitalWrite(motors[motor_index].dir_b_pin, LOW);
    } else if (value < 0) {
        motor_control_internals::digitalWrite(motors[motor_index].dir_a_pin, LOW);
        motor_control_internals::digitalWrite(motors[motor_index].dir_b_pin, HIGH);
    } else {
        motor_control_internals::digitalWrite(motors[motor_index].dir_a_pin, LOW);
        motor_control_internals::digitalWrite(motors[motor_index].dir_b_pin, LOW);
    }
}

void rampCurrent(uint8_t motor_index) {
    if (motors[motor_index].current == motors[motor_index].target) {
        return;
    }

    // An H-bridge must reach zero before its direction pins are reversed.
    if (motors[motor_index].hbridge && 
        motors[motor_index].current != 0 && 
        ((motors[motor_index].current > 0) != (motors[motor_index].target > 0))) {
        
        int16_t step = motors[motor_index].acceleration_step;
        int16_t distance_to_zero = mabs(motors[motor_index].current);
        if (step >= distance_to_zero) {
            motors[motor_index].current = 0;
        } else {
            motors[motor_index].current += motors[motor_index].current > 0 ? -step : step;
        }
        if (motors[motor_index].current == 0) {
            writeDirection(motor_index, 0);
        }
        return;
    }

    int16_t step = motors[motor_index].acceleration_step;
    int32_t next_current = motors[motor_index].current;
    if (motors[motor_index].current < motors[motor_index].target) {
        next_current += step;
        if (next_current > motors[motor_index].target) {
            next_current = motors[motor_index].target;
        }
    } else {
        next_current -= step;
        if (next_current < motors[motor_index].target) {
            next_current = motors[motor_index].target;
        }
    }
    motors[motor_index].current = static_cast<int16_t>(next_current);
    writeDirection(motor_index, motors[motor_index].current);
}

void scheduleNextCompare() {
    if (current_off_order_index >= off_order_count) {
        return;
    }

    uint8_t motor_index = off_order[current_off_order_index];
    uint16_t target_ocr = mabs(motors[motor_index].current) << 6; // '<< 6' ~ '* PWM_PERIOD_TICKS / 1000';
    uint16_t safe_ocr = TCNT1 + TIMER_GUARD_TICKS;

    // Do not schedule a compare event too close to the current timer value.
    OCR1B = target_ocr > safe_ocr ? target_ocr : safe_ocr;
}

}

namespace dc_control_internal {

int8_t registerMotor(uint8_t pwm_pin, uint8_t dir_a_pin, uint8_t dir_b_pin, bool hbridge) {
    for (uint8_t index = 0; index < MAX_MOTOR_COUNT; ++index) {
        if (!motors[index].active) {
            motors[index].active = true;
            motors[index].hbridge = hbridge;
            motors[index].pwm_pin = static_cast<int8_t>(pwm_pin);
            motors[index].dir_a_pin = dir_a_pin;
            motors[index].dir_b_pin = dir_b_pin;
            motors[index].target = 0;
            motors[index].current = 0;
            motors[index].acceleration_step = 1;
            writeDirection(index, 0);
            return static_cast<int8_t>(index);
        }
    }
    return -1;
}

void setTarget(int8_t motor_index, int16_t value) {
    if (motor_index < 0 || motor_index >= MAX_MOTOR_COUNT) {
        return;
    }
    if (value > 1000) value = 1000;
    if (value < -1000) value = -1000;

    cli();
    motors[motor_index].target = value;
    sei();
}

void setImmediate(int8_t motor_index, int16_t value) {
    if (motor_index < 0 || motor_index >= MAX_MOTOR_COUNT) {
        return;
    }
    if (value > 1000) value = 1000;
    if (value < -1000) value = -1000;

    cli();
    motors[motor_index].current = value;
    motors[motor_index].target = value;
    sei();
    writeDirection(static_cast<uint8_t>(motor_index), value);
}

void setAcceleration(int8_t motor_index, uint16_t percent_per_decisecond) {
    if (motor_index < 0 || motor_index >= MAX_MOTOR_COUNT) {
        return;
    }
    if (percent_per_decisecond < 25) {
        percent_per_decisecond = 25;
    } else if (percent_per_decisecond > 5000) {
        percent_per_decisecond = 5000;
    }

    // Timer1 overflows every 4 ms, so 100 ms contains 25 ramp steps.
    motors[motor_index].acceleration_step = percent_per_decisecond / 25;
}

void onTimerOverflow() {
    off_order_count = 0;

    for (uint8_t index = 0; index < MAX_MOTOR_COUNT; ++index) {
        if (!motors[index].active) {
            continue;
        }

        rampCurrent(index);
        if (motors[index].current != 0) {
            motor_control_internals::digitalWrite(motors[index].pwm_pin, HIGH);
        }
        off_order[off_order_count++] = index;
    }

    // A single timer channel serves all motors: sort their falling edges.
    for (uint8_t i = 0; i < off_order_count; ++i) {
        for (uint8_t j = i + 1; j < off_order_count; ++j) {
            if (mabs(motors[off_order[j]].current) <
                mabs(motors[off_order[i]].current)) {
                uint8_t temporary = off_order[i];
                off_order[i] = off_order[j];
                off_order[j] = temporary;
            }
        }
    }

    current_off_order_index = 0;
    scheduleNextCompare();
}

void onTimerCompareMatch() {
    if (current_off_order_index >= off_order_count) {
        return;
    }

    uint8_t motor_index = off_order[current_off_order_index];
    motor_control_internals::digitalWrite(motors[motor_index].pwm_pin, LOW);
    ++current_off_order_index;
    scheduleNextCompare();
}

}

// TimerControl calls these functions from its Timer1 ISR.
void OnTimer1OwerflowDC() {
    dc_control_internal::onTimerOverflow();
}

void OnTimer1CompareMatchDC() {
    dc_control_internal::onTimerCompareMatch();
}
