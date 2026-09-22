#ifndef DC_CONTROL_H
#define DC_CONTROL_H

#include <stdint.h>

class DCControl {
public:
    /** Creates an uninitialized driver. Call init() before using it. */
    DCControl();

    /**
     * Creates a one-pin DC PWM driver.
     * @param pin_pwm Arduino pin used as the software-PWM output.
     */
    DCControl(uint8_t pin_pwm);

    /**
     * Registers a software-PWM output.
     * @param pin_pwm Arduino pin used as the PWM output.
     * @return ErrorCodes::NO_ERROR or ErrorCodes::ERROR_NO_FREE_MOTOR.
     */
    uint8_t init(uint8_t pin_pwm);

    /**
     * Sets the requested PWM duty cycle and applies the configured ramp.
     * @param value Duty cycle from 0 (off) to 1000 (full duty cycle).
     *              Values above 1000 are clamped to 1000.
     */
    void setTarget(uint16_t value);
    
    /**
     * Sets the PWM duty cycle without ramping.
     * @param value Duty cycle from 0 (off) to 1000 (full duty cycle).
     *              Values above 1000 are clamped to 1000.
     */
    void setImmediate(uint16_t value);

    /**
     * Sets the PWM ramp rate.
     * @param percent_per_decisecond Change of the 0..1000 PWM value per
     *        100 ms. Valid range is 25..5000; values outside it are clamped.
     */
    void setAcceleration(uint16_t percent_per_decisecond);

    /** Returns the last initialization error for this instance. */
    uint8_t getLastError();

  
private:
    int8_t motor_index_;
    uint8_t error_code_;
};

#ifndef MOTOR_CONTROL_ERROR_CODES_DEFINED
#define MOTOR_CONTROL_ERROR_CODES_DEFINED
namespace ErrorCodes {
    enum ErrorCode {
        NO_ERROR = 0,
        ERROR_NO_FREE_MOTOR = -1,
        ERROR_INVALID_PIN = -2,
        ERROR_INVALID_VALUE = -3
};
}
#endif

#endif // DC_CONTROL_H
