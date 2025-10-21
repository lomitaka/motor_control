#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <avr/io.h>
#include <stddef.h>
#include <stdint.h>
#include "motor.h"

/*
    //how it works (Servo approach)
    - Timer 1 is set to 4ms overflow period (16MHz / 1 prescaler / 65536 counts = 244 Hz → 4.096 ms)
    each cycle one servo motor is handled (5 motors max)
     on the beginning of the cycle the pin is set HIGH, and compare match is set according to motor value
        when compare match occurs, pin is set LOW
    - thus each motor gets a pulse every 20ms (5 motors x 4ms)
    
       
    
*/

class ServoControl {
public:
    ServoControl();



    ServoControl(uint8_t pin);

    ~ServoControl() ;
    // Set target: for DC/stepper -> speed (-1000..1000), for servo -> angle (radians) depending on implementation
    void setTarget(int16_t value);
    
    // Immediately set output (no ramp) — useful for calibration/emergency
    void setImmediate(int16_t value);

    // Configure ramping: type and time to reach target (milliseconds)
    void configureRamp(int16_t rampTime_ms);
    
    //in case of failure, get last error code
    int getLastError();

private:
    
    

    static  void setup_Timers();

    // Set motor value in range [-1000, 1000] for given index (0-4)
    static void setServMotorValue(uint8_t index, int16_t value);

    // Set port and pin for given motor index (0-4), port: 1=A, 2=B, 3=C, 4=D, pin: 0-7
    static void setServMotorPortPin(uint8_t index, uint8_t port, uint8_t pin);
    // Overloaded version: set port and pin using single byte (port in high nibble, pin in low nibble)
    static void setServMotorPortPin(uint8_t index, uint8_t port_pin);

    // Get time since start
    static size_t getTimeSinceStart();
    // get index of first free motor
    static int8_t getServFreeMotorIndex();
    
    //detaches motor from given index
    void freeServIndex(uint8_t index);

    //how many motors are registered
    volatile static uint8_t serv_motor_count_;
    //motor values currently set (-1000 to 1000)
    volatile static uint16_t serv_motors_[5];
    //port and pin for each motor (0=disabled, else port in high nibble, pin in low nibble)
    volatile static uint8_t serv_port_pin_[5];

    friend void OnTimer1CompareMatchServo();
    friend void OnTimer1OwerflowServo();

    MotorProps props;
};

#endif // SERVO_CONTROL_H
