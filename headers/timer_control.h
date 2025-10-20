#ifndef TIMER_CONTROL_H
#define TIMER_CONTROL_H

#include <avr/io.h>
#include <stddef.h>
#include <stdint.h>

/*
    //how it works (Servo approach)
    - Timer 1 is set to 4ms overflow period (16MHz / 1 prescaler / 65536 counts = 244 Hz → 4.096 ms)
    each cycle one servo motor is handled (5 motors max)
     on the beginning of the cycle the pin is set HIGH, and compare match is set according to motor value
        when compare match occurs, pin is set LOW
    - thus each motor gets a pulse every 20ms (5 motors x 4ms)
    
       
    
*/

class TimerControl {
public:
    TimerControl();

    static  void setup_Timers();

    // Set motor value in range [-1000, 1000] for given index (0-4)
    static void setMotorValue(uint8_t index, int16_t value);

    // Set port and pin for given motor index (0-4), port: 1=A, 2=B, 3=C, 4=D, pin: 0-7
    static void setMotorPortPin(uint8_t index, uint8_t port, uint8_t pin);
    // Overloaded version: set port and pin using single byte (port in high nibble, pin in low nibble)
    static void setMotorPortPin(uint8_t index, uint8_t port_pin);

    // Get time since start
    static size_t getTimeSinceStart();
    // get index of first free motor
    static int8_t getFreeMotorIndex();
    //detaches motor from given index
    void freeIndex(uint8_t index);
    //in case of failure, get last error code
    int getLastError();

  
private:
    volatile size_t timeSinceStart_;
    volatile static uint8_t curr_motor_i;
    //how many motors are registered
    volatile static uint8_t serv_motor_count_;
    //motor values currently set (-1000 to 1000)
    volatile static uint16_t serv_motors_[5];
    //port and pin for each motor (0=disabled, else port in high nibble, pin in low nibble)
    volatile static uint8_t serv_port_pin_[5];

    //dc motors:
    volatile static uint8_t dc_motor_count_;
    //motor values currently set (-1000 to 1000)
    volatile static uint16_t dc_motors_[5];
    //port and pin for each motor (0=disabled, else port in high nibble, pin in low nibble)
    volatile static uint8_t dc_port_pin_[5];


    //last error codes for each motor
    volatile static uint8_t last_error_[5];
    //global last error code
    volatile static uint8_t last_error_global;

    static void Timer1_Init();
    static  void setPinHigh(uint8_t port_pin_code);
    static  void setPinLow(uint8_t port_pin_code);

    friend void TIMER1_COMPA_vect();
    friend void TIMER1_OVF_vect();
};

extern TimerControl timer_control;
#endif // TIMER_CONTROL_H
