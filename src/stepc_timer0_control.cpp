#include "internals/stepc_timer_control.h"

#ifdef SIMULATION_MODE
    #include "simulator/avr_mock.h"
#else
    #include <avr/io.h>
    #include <avr/interrupt.h>
#endif

bool StepCTimerControl::initialized_ = false;

/**
 * Configuration summary:
 * - Mode: CTC (WGM01 = 1), counter resets at Compare Match A.
 * - Clock: 16 MHz / 256 = 62.5 kHz, 16 us timer tick.
 * - OCR0A: 7, which is 8 timer counts per CTC period on AVR.
 * - Period: 128 us; Compare Match A ISR frequency: 7812.5 Hz.
 * - Interrupts: Compare Match A only.
 */
void StepCTimerControl::Timer1_Init() {
    uint8_t saved_sreg = SREG;
    cli();

    // Clear timer configuration
    TCCR0A = 0;
    TCCR0B = 0;
    TCNT0 = 0;

    // Set compare value for a 128 us period.
    // Timer freq = 16 MHz / 256 = 62.5 kHz
    // CTC counts from 0 through OCR0A: (7 + 1) * 16 us = 128 us.
    OCR0A = 7;

    // Enable Timer0 Compare Match A interrupt
    TIMSK0 = (1 << OCIE0A);

    // Configure Timer0:
    // - CTC mode (WGM01=1): Clear Timer on Compare Match A
    // - Prescaler 256 (CS02=1): Timer runs at CPU_freq / 256
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS02);

    initialized_ = true;
    SREG = saved_sreg;
}

bool StepCTimerControl::isInitialized() {
    return initialized_;
}

ISR(TIMER0_COMPA_vect) {
    OnTimer1StepperContinuousISR();
}

