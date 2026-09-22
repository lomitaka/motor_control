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

enum class TimerMode : uint8_t {
    Unconfigured,
    DcServo,
    StepperContinuous,
    StepperPositioning
};

/*
 * Timer1 runs in CTC mode with a 4 ms period at 16 MHz.
 * TIMER1_COMPA_vect starts each software-PWM period and TIMER1_COMPB_vect
 * creates falling edges for DC PWM and servo pulses. Five servo slots are
 * multiplexed over five periods, so each servo receives one pulse per 20 ms.
 */

class TimerControl {
public:

    // Configures Timer1 but deliberately does not call sei().
    // Enable global interrupts from main() after application initialization.
    static void setup_Timers();
    static bool isInitialized();

    /** Selects the driver family currently owning Timer1. */
    static void setTimerMode(TimerMode mode);

    /** Returns the driver family currently owning Timer1. */
    static TimerMode getTimerMode();

    // Returns the most recent TimerControl error code.
    static int getLastError();
    
    // port_pin_code stores the port in the high nibble and pin in the low nibble.
    static void setPinHigh(uint8_t port_pin_code);
    static void setPinLow(uint8_t port_pin_code);
  
//private:
    volatile size_t timeSinceStart_;


    // Last error codes for each motor.
    volatile static uint8_t last_error_[5];

    static void Timer1_Init();
    static bool initialized;
    static volatile TimerMode timer_mode_;

    friend void OnTimer1CompareMatchDC();
    friend void OnTimer1OwerflowDC();
    friend void OnTimer1CompareMatchServo();
    friend void OnTimer1OwerflowServo();
};



//extern TimerControl timer_control;
#endif // TIMER_CONTROL_H
