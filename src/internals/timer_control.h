#ifndef TIMER_CONTROL_H
#define TIMER_CONTROL_H

#ifdef SIMULATION_MODE
    #include "simulator/avr_mock.h"
#else
    #include <avr/io.h>
#endif
//#include <avr/io.h>
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

    static  void setup_Timers();
    static bool isInitialized();

    //in case of failure, get last error code
    static int getLastError();
    
    static  void setPinHigh(uint8_t port_pin_code);
    static  void setPinLow(uint8_t port_pin_code);
  
//private:
    volatile size_t timeSinceStart_;


    //dc motors:
    volatile static uint8_t curr_dc_index;
    volatile static uint16_t curr_dc_value;


    //last error codes for each motor
    volatile static uint8_t last_error_[5];
    //global last error code
    volatile static uint8_t last_error_global;

    static void Timer1_Init();
    static bool initialized;

    friend void OnTimer1CompareMatchDC();
    friend void OnTimer1OwerflowDC();
    friend void OnTimer1CompareMatchServo();
    friend void OnTimer1OwerflowServo();
};



//extern TimerControl timer_control;
#endif // TIMER_CONTROL_H
