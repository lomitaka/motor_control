#ifndef DC_CONTROL_HBRIDGE_H
#define DC_CONTROL_HBRIDGE_H

#include <stdint.h>

class DCControlHBridge {
public:
    /** Creates an uninitialized H-bridge driver. Call init() before using it. */
    DCControlHBridge();

    /**
     * Creates an H-bridge driver.
     * @param pin_pwm Arduino pin used as the software-PWM output.
     * @param pin_a First direction input of the H-bridge.
     * @param pin_b Second direction input of the H-bridge.
     */
    DCControlHBridge(uint8_t pin_pwm, uint8_t pin_a, uint8_t pin_b);

    /**
     * Registers PWM and direction pins for an H-bridge.
     * @return ErrorCodes::NO_ERROR or ErrorCodes::ERROR_NO_FREE_MOTOR.
     */
    uint8_t init(uint8_t pin_pwm, uint8_t pin_a, uint8_t pin_b);

    /**
     * Sets requested duty cycle and direction using the configured ramp.
     * @param value Range -1000..1000. Zero stops the motor; magnitude is the
     *        PWM duty cycle and sign selects direction. Values outside the
     *        range are clamped. A direction reversal ramps through zero.
     */
    void setTarget(int16_t value);

    /**
     * Sets duty cycle and direction immediately, without the reversal ramp.
     * @param value Range -1000..1000; magnitude is PWM duty cycle and sign
     *        selects direction. Values outside the range are clamped.
     */
    void setImmediate(int16_t value);

    /**
     * Sets the ramp rate for acceleration and deceleration.
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

using DCControlHBRidge = DCControlHBridge;

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
