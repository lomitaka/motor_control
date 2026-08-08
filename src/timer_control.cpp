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

//singleton implementation of timer control (to controll timer1)
//TimerControl timer_control;

// Definice static členských proměnných

//volatile uint16_t TimerControl::curr_dc_value = 0;
volatile uint8_t TimerControl::last_error_[5] = {0, 0, 0, 0, 0};
volatile uint8_t TimerControl::last_error_global = 0;
bool TimerControl::initialized = false;



bool  TimerControl::isInitialized(){
    return initialized;
}

int TimerControl::getLastError()
{
    return last_error_global;
}



/// Initializes Timer1 for precise timing control.
/// 
/// Configuration summary:
/// - Timer1 runs in **CTC mode** (Clear Timer on Compare Match A).
/// - Compare Match A interrupt (OCIE1A), Compare Match B (OCIE1B), and Overflow interrupt (TOIE1) are enabled.
/// - Compare value (OCR1A = 63999) defines the period (timer resets at this value).
/// - No prescaler is used (CS10 = 1), so timer runs at full CPU clock speed.
///
/// For example, with a 16 MHz clock:
///   - Each timer tick = 1 / 16,000,000 s = 62.5 ns
///   - OCR1A = 63999 → CTC period = 64000 × 62.5 ns = 4 ms
///   - PWM frequency = 250 Hz
///   - Compare Match A defines the period (timer resets to 0)
///   - Compare Match B is dynamically scheduled for motor PWM falling edges
///
/// This setup generates software PWM for DC motors and servo control.
/// (Compare Match B triggers ISR to set pins LOW at duty cycle points,
///  Overflow ISR sets pins HIGH for the next PWM cycle.)
void TimerControl::Timer1_Init() {
    
    TCNT1 = 0;                           // Reset timer counter
    OCR1A = 63999;                       //
    OCR1B = 65535;                       // Set initial compare value so it wont trigger before first overflow
    TIMSK1 = (1 << OCIE1A) |  (1 << OCIE1B)  | (1 << TOIE1); // Enable Compare A,B Match and Overflow interrupts
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
    sei();
    
}


void OnTimer1CompareMatchDC();
void OnTimer1CompareMatchServo();

// ISRs must be outside the class, but can call static member functions or access static members
ISR(TIMER1_COMPB_vect) {	
    //OnTimer1CompareMatchDC();
    OnTimer1CompareMatchServo();
    OnTimer1CompareMatchDC();
    
}

void OnTimer1OwerflowServo();
void OnTimer1OwerflowDC();

extern char serv_dbg[64];

ISR(TIMER1_OVF_vect) {
    OnTimer1OwerflowServo();
    OnTimer1OwerflowDC();

}