#include "internals/stepp_timer_control.h"

#ifdef SIMULATION_MODE
    #include "simulator/avr_mock.h"
#else
    #include <avr/io.h>
    #include <avr/interrupt.h>
#endif

bool SteppTimerControl::initialized_ = false;

void SteppTimerControl::Timer1_Init() {
    // Clear timer configuration
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    
    // Set initial compare values
    OCR1A = 64000;  // Initial value (will be updated dynamically)
    OCR1B = 65535;  // Initial value (will be updated dynamically)
    
    // Enable Timer1 Compare Match A and B interrupts
    TIMSK1 = (1 << OCIE1A) | (1 << OCIE1B);
    
    // Configure Timer1:
    // - CTC mode (WGM12=1): Clear Timer on Compare Match A
    // - No prescaler (CS10=1): Timer runs at CPU_freq (16 MHz)
    TCCR1B = (1 << WGM12) | (1 << CS10);
    
    initialized_ = true;
}

bool SteppTimerControl::isInitialized() {
    return initialized_;
}

// Timer1 Compare Match A ISR - rising edges and scheduling
#ifdef SIMULATION_MODE
ISR(TIMER1_COMPA_vect) {
    OnTimer1StepperPositioningOCRA();
}
#else
ISR(TIMER1_COMPA_vect) {
    OnTimer1StepperPositioningOCRA();
}
#endif

// Timer1 Compare Match B ISR - falling edges
#ifdef SIMULATION_MODE
ISR(TIMER1_COMPB_vect) {
    OnTimer1StepperPositioningOCRB();
}
#else
ISR(TIMER1_COMPB_vect) {
    OnTimer1StepperPositioningOCRB();
}
#endif
