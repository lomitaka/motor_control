#include "internals/timer_control.h"
#ifdef SIMULATION_MODE
    #include "simulator/avr_mock.h"
#else
    #include <avr/io.h>
    #include <avr/interrupt.h>
    #include "util/delay.h"
#endif

//#include <avr/io.h>
//#include <avr/interrupt.h>
#define __DELAY_BACKWARD_COMPATIBLE__

#include "stddef.h"

// Shared Timer1 configuration state.
volatile uint8_t TimerControl::last_error_[5] = {0, 0, 0, 0, 0};
bool TimerControl::initialized = false;



bool  TimerControl::isInitialized(){
    return initialized;
}


/// Initializes Timer1 for DC motor and servo software PWM.
///
/// Configuration summary:
/// - Mode: CTC (WGM12 = 1), counter resets at Compare Match A.
/// - Clock: 16 MHz, no prescaler (CS10 = 1), 62.5 ns timer tick.
/// - OCR1A: 63999, giving a 4 ms PWM period (250 Hz).
/// - Interrupts: Compare A starts each period; Compare B creates PWM falling edges.
/// - Servo pulses are multiplexed over five 4 ms periods, yielding a 20 ms refresh period.
void TimerControl::Timer1_Init() {
    
    TCNT1 = 0;                           // Reset timer counter.
    OCR1A = 63999;                       // 64,000 ticks at 16 MHz = 4 ms.
    OCR1B = 63999;                       // Avoid a Compare B event before the first period callback.
    TIMSK1 = (1 << OCIE1A) | (1 << OCIE1B) | (1 << TOIE1);
    TCCR1A = 0;                          // Normal mode (no PWM)
    TCCR1B =  (1 << WGM12) | (1 << CS10);//  Clears timer on Compare match A, Prescaler = 1 → timer runs at full CPU speed
}

void TimerControl::setPinHigh(uint8_t port_pin_code) {
    uint8_t port = (port_pin_code >> 4) & 0x0F;
    uint8_t pin = port_pin_code & 0x0F;
    switch (port) {
    //case 1: PORTA |= (1 << pin); break;
    case 2: PORTB |= (1 << pin); break;
    case 3: PORTC |= (1 << pin); break;
    case 4: PORTD |= (1 << pin); break;
    default: break;
    }
}

void TimerControl::setPinLow(uint8_t port_pin_code) {
    uint8_t port = (port_pin_code >> 4) & 0x0F;
    uint8_t pin = port_pin_code & 0x0F;

    switch (port) {
    //case 1: PORTA &= ~(1 << pin); break;
    case 2: PORTB &= ~(1 << pin); break;
    case 3: PORTC &= ~(1 << pin); break;
    case 4: PORTD &= ~(1 << pin); break;
    default: break;
    }
}

void TimerControl::setup_Timers() {
    Timer1_Init();
    initialized = true;
}


void OnTimer1CompareMatchDC();
void OnTimer1CompareMatchServo();

// AVR interrupt vectors must be free functions rather than class members.
ISR(TIMER1_COMPB_vect) {	
    OnTimer1CompareMatchServo();
    OnTimer1CompareMatchDC();
}

void OnTimer1OwerflowServo();
void OnTimer1OwerflowDC();

// In CTC mode Compare A is the PWM period boundary, not a hardware overflow.
ISR(TIMER1_COMPA_vect) {
    OnTimer1OwerflowServo();
    OnTimer1OwerflowDC();
}