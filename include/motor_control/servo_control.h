#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#ifdef SIMULATION_MODE
    #include "simulator/avr_mock.h"
#else
    #include <avr/io.h>
    #include <avr/interrupt.h>
#endif

//#include <avr/io.h>
#include <stddef.h>
#include <stdint.h>

class ServoControl {
public:

    /**
     * Creates and registers a servo output.
     * @param pin Arduino pin used for the servo pulse output.
     */
    ServoControl(uint8_t pin);


    /**
     * Sets the requested servo pulse value.
     * @param value Nominal range -1000..1000 maps approximately to 1..2 ms.
     *        The implementation accepts and clamps -2000..2000, extending the
     *        generated pulse range beyond the nominal servo range.
     */
    void setTarget(int16_t value);
    
    /**
     * Sets the servo pulse value immediately.
     * @param value Same range and clamping as setTarget(). The current
     *        implementation has no separate servo ramp, so this behaves the
     *        same as setTarget().
     */
    void setImmediate(int16_t value);

    

private:
    
    // Set motor value in range [-1000, 1000] for given index (0-4)
    static void setServMotorValue(uint8_t index, int16_t value);

    // Set port and pin for given motor index (0-4), port: 1=A, 2=B, 3=C, 4=D, pin: 0-7
    static void setServMotorPortPin(uint8_t index, uint8_t pin);

    // get index of first free motor
    static int8_t getServFreeMotorIndex();
    
    //detaches motor from given index
    void freeServIndex(uint8_t index);
//public:
    //how many motors are registered
    volatile static uint8_t serv_motor_count_;
    //motor values currently set (-1000 to 1000)
    volatile static uint16_t serv_motors_[5];
    //port and pin for each motor (0=disabled, else port in high nibble, pin in low nibble)
    volatile static uint8_t serv_pin_[5];

    friend void OnTimer1CompareMatchServo();
    friend void OnTimer1OwerflowServo();

    //MotorProps props;
    volatile static uint8_t curr_motor_i;
    uint8_t port_pin_;
    int8_t motor_index;
};

#endif // SERVO_CONTROL_H
