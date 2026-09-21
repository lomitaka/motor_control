#ifndef DC_SCHEDULER_H
#define DC_SCHEDULER_H

#include <stdint.h>

namespace dc_control_internal {

int8_t registerMotor(uint8_t pwm_pin, uint8_t dir_a_pin, uint8_t dir_b_pin, bool hbridge);
void setTarget(int8_t motor_index, int16_t value);
void setImmediate(int8_t motor_index, int16_t value);
void setAcceleration(int8_t motor_index, uint16_t percent_per_decisecond);
void onTimerOverflow();
void onTimerCompareMatch();

}

#endif
