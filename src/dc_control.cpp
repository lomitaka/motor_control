#include "motor_control/dc_control.h"
#include "internals/arduino.h"
#include "internals/dc_scheduler.h"
#include "internals/timer_control.h"

DCControl::DCControl()
    : motor_index_(-1), error_code_(ErrorCodes::NO_ERROR) {
}

DCControl::DCControl(uint8_t pin_pwm)
    : DCControl() {
    init(pin_pwm);
}

uint8_t DCControl::init(uint8_t pin_pwm) {
    // A basic DC motor uses only one output: the scheduler generates PWM on it.
    motor_control_internals::pinMode(pin_pwm, OUTPUT);
    motor_index_ = dc_control_internal::registerMotor(pin_pwm, 0, 0, false);
    if (motor_index_ < 0) {
        error_code_ = ErrorCodes::ERROR_NO_FREE_MOTOR;
        return error_code_;
    }

    // The scheduler configures Timer1, but interrupts are enabled by main().
    if (!TimerControl::isInitialized()) {
        TimerControl::setup_Timers();
    }
    return ErrorCodes::NO_ERROR;
}

void DCControl::setTarget(uint16_t value) {
    if (value > 1000) {
        value = 1000;
    }
    dc_control_internal::setTarget(motor_index_, static_cast<int16_t>(value));
}

void DCControl::setImmediate(uint16_t value) {
    if (value > 1000) {
        value = 1000;
    }
    dc_control_internal::setImmediate(motor_index_, static_cast<int16_t>(value));
}

void DCControl::setAcceleration(uint16_t percent_per_decisecond) {
    dc_control_internal::setAcceleration(motor_index_, percent_per_decisecond);
}

uint8_t DCControl::getLastError() {
    return error_code_;
}
