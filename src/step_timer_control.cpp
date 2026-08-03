#include "internals/step_timer_control.h"

#ifdef SIMULATION_MODE
    #include "simulator/avr_mock.h"
#else
    #include <avr/io.h>
    #include <avr/interrupt.h>
#endif

bool StepTimerControl::initialized_ = false;

void StepTimerControl::Timer1_Init() {
    // Clear timer configuration
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    
    // Set compare value for 112μs period
    // Timer freq = 16 MHz / 256 = 62.5 kHz
    // 112μs = 7 ticks at 16μs per tick
    OCR1A = 7;
    
    // Enable Timer1 Compare Match A interrupt
    TIMSK1 = (1 << OCIE1A);
    
    // Configure Timer1:
    // - CTC mode (WGM12=1): Clear Timer on Compare Match A
    // - Prescaler 256 (CS12=1): Timer runs at CPU_freq / 256
    TCCR1B = (1 << WGM12) | (1 << CS12);
    
    initialized_ = true;
}

bool StepTimerControl::isInitialized() {
    return initialized_;
}

// Timer1 Compare Match A ISR - calls stepper motor handler
#ifdef SIMULATION_MODE
ISR(TIMER1_COMPA_vect) {
    OnTimer1StepperISR();
}
#else
ISR(TIMER1_COMPA_vect) {
    OnTimer1StepperISR();
}
#endif
