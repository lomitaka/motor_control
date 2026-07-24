#pragma once
#include <stdint.h>
#include "motor_control/ramp.h"

namespace ErrorCodes {
    enum ErrorCode {
        NO_ERROR = 0,
        ERROR_NO_FREE_MOTOR = -1,
        ERROR_INVALID_PIN = -2,
        ERROR_INVALID_VALUE = -3
};
}

// basic state reported by motor
struct MotorState {
    int16_t currentSetpoint; // -1000..1000 for speed, or radians for servo depending on mode
    int16_t targetSetpoint;
    bool running;
};

class IMotor {
public:

    virtual ~IMotor() {};

    // Initialize motor with a specific value
    //returns error code if failed or 0 if success
    virtual uint8_t init(uint8_t pin) = 0;

    // Set target: for DC/stepper -> speed (-1000..1000), for servo -> angle (degrees) depending on implementation
    virtual void setTarget(int16_t value) = 0;
    
    // Immediately set output (no ramp) — useful for calibration/emergency
    virtual void setImmediate(int16_t value) = 0;

    // Configure ramping: type and time to reach target (milliseconds)
    virtual void configureRamp(int16_t rampTime_ms) = 0;

    virtual uint8_t getLastError();
    
};



/*class MotorProps {
public:
    int16_t motor_index = -1;
    uint8_t port_pin_ =0;
    MotorState state_;
    Ramp ramp_;
    int16_t currentOutput_; // current output value, ramped
    int16_t targetOutput_;  // target output value
    uint8_t error_code_ = 0;
};*/