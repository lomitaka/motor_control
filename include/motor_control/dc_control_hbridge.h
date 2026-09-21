#ifndef DC_CONTROL_HBRIDGE_H
#define DC_CONTROL_HBRIDGE_H

#include <stdint.h>

class DCControlHBridge {
public:
    DCControlHBridge();

    DCControlHBridge(uint8_t pin_pwm, uint8_t pin_a, uint8_t pin_b);

    uint8_t init(uint8_t pin_pwm, uint8_t pin_a, uint8_t pin_b);

    void setTarget(int16_t value);

    void setImmediate(int16_t value);

    // Sets ramp speed in percent per 100 ms. Valid range: 25..5000.
    void setAcceleration(uint16_t percent_per_decisecond);

    //in case of failure, get last error code
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
