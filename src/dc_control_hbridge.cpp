#include "motor_control/dc_control_hbridge.h"
#include "internals/arduino.h"
#include "internals/dc_scheduler.h"
#include "internals/timer_control.h"

DCControlHBridge::DCControlHBridge()
    : motor_index_(-1), error_code_(ErrorCodes::NO_ERROR) {
}

DCControlHBridge::DCControlHBridge(uint8_t pin_pwm, uint8_t pin_a, uint8_t pin_b)
    : DCControlHBridge() {
    init(pin_pwm, pin_a, pin_b);
}

uint8_t DCControlHBridge::init(uint8_t pin_pwm, uint8_t pin_a, uint8_t pin_b) {
    // The shared scheduler owns PWM timing; this class only supplies H-bridge pins.
    motor_control_internals::pinMode(pin_pwm, OUTPUT);
    motor_control_internals::pinMode(pin_a, OUTPUT);
    motor_control_internals::pinMode(pin_b, OUTPUT);
    motor_index_ = dc_control_internal::registerMotor(pin_pwm, pin_a, pin_b, true);
    if (motor_index_ < 0) {
        error_code_ = ErrorCodes::ERROR_NO_FREE_MOTOR;
        return error_code_;
    }

    if (!TimerControl::isInitialized()) {
        TimerControl::setup_Timers();
    }
    return ErrorCodes::NO_ERROR;
}

void DCControlHBridge::setTarget(int16_t value) {
    // Positive and negative values select opposite bridge directions.
    dc_control_internal::setTarget(motor_index_, value);
}

void DCControlHBridge::setImmediate(int16_t value) {
    // Immediate changes bypass the ramp and are intended for emergency control.
    dc_control_internal::setImmediate(motor_index_, value);
}

void DCControlHBridge::setAcceleration(uint16_t percent_per_decisecond) {
    dc_control_internal::setAcceleration(motor_index_, percent_per_decisecond);
}

uint8_t DCControlHBridge::getLastError() {
    return error_code_;
}
