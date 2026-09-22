#include "internals/step_timer_control.h"

#ifdef SIMULATION_MODE
    #include "simulator/avr_mock.h"
#else
    #include <avr/io.h>
    #include <avr/interrupt.h>
#endif

bool StepCTimerControl::initialized_ = false;

/**
 * Configuration summary:
 * - Mode: CTC (WGM12 = 1), counter resets at Compare Match A.
 * - Clock: 16 MHz / 256 = 62.5 kHz, 16 us timer tick.
 * - OCR1A: 7, which is 8 timer counts per CTC period on AVR.
 * - Period: 128 us; Compare Match A ISR frequency: 7812.5 Hz.
 * - Interrupts: Compare Match A only.
 */
void StepCTimerControl::Timer1_Init() {
    // Clear timer configuration
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    
    // Set compare value for a 128 us period.
    // Timer freq = 16 MHz / 256 = 62.5 kHz
    // CTC counts from 0 through OCR1A: (7 + 1) * 16 us = 128 us.
    OCR1A = 7;
    
    // Enable Timer1 Compare Match A interrupt
    TIMSK1 = (1 << OCIE1A);
    
    // Configure Timer1:
    // - CTC mode (WGM12=1): Clear Timer on Compare Match A
    // - Prescaler 256 (CS12=1): Timer runs at CPU_freq / 256
    TCCR1B = (1 << WGM12) | (1 << CS12);
    
    initialized_ = true;
}

bool StepCTimerControl::isInitialized() {
    return initialized_;
}

// Timer1 Compare Match A ISR - calls stepper motor handler
#ifdef SIMULATION_MODE
ISR(TIMER1_COMPA_vect) {
    OnTimer1StepperContinuousISR();
}
#else
ISR(TIMER1_COMPA_vect) {
    OnTimer1StepperContinuousISR();
}
#endif
