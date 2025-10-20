#include "headers/timer_control.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#define __DELAY_BACKWARD_COMPATIBLE__
#include "util/delay.h"
#include "headers/usart.h"
#include "stddef.h"
#include "headers/logic.h"
#include "timer_control.h"

//singleton implementation of timer control (to controll timer1)
TimerControl timer_control;

/*size_t TimerControl::timeSinceStart_ = 0;
//index of currently active motor (0-4)
uint8_t TimerControl::curr_motor_i = 0;
//number of motors registered (0-5)
uint8_t TimerControl::serv_motor_count_ = 0;
//motor values currently set (-1000 to 1000)
int16_t TimerControl::serv_motors_[5] = {0,0,0,0,0};
//port and pin for each motor (0=disabled, else port in high nibble, pin in low nibble)
uint8_t TimerControl::serv_port_pin_[5] = {0x00, 0x00, 0x00, 0x00, 0x00};*/

int8_t TimerControl::getFreeMotorIndex(){
    for (int8_t i = 0; i < 5; i++) {
        if (serv_port_pin_[i] == 0) {
            return i;
        }
    }
    return -1; // No free motor index available
}

void TimerControl::freeIndex(uint8_t index){
    serv_port_pin_[index] = 0;
    serv_motors_[index] = 0;
    
}

int TimerControl::getLastError()
{
    return last_error_global;
}
void TimerControl::setMotorValue(uint8_t index, int16_t value)
{
    if (index < 5){
        if (value > 1000) value = 1000;
        if (value < -1000) value = -1000;
        serv_motors_[index] = value;
    }
}

    // Set port and pin for given motor index (0-4), port: 1=A, 2=B, 3=C, 4=D, pin: 0-7
    //set port to 0 to disable motor
void TimerControl::setMotorPortPin(uint8_t index, uint8_t port, uint8_t pin){
    if (index < 5 && port <= 4 && pin <= 7){
        serv_port_pin_[index] = (port << 4) | (pin & 0x0F);
    }
}

    // Set port and pin for given motor index (0-4), port: 1=A, 2=B, 3=C, 4=D, pin: 0-7
    //set port to 0 to disable motor
void TimerControl::setMotorPortPin(uint8_t index, uint8_t port_pin ){
    if (index < 5 && port_pin <= 0x4F){
        serv_port_pin_[index] = port_pin;
    }
}


/// Initializes Timer1 for precise timing control.
/// 
/// Configuration summary:
/// - Timer1 runs in **Normal mode** (counts from 0 to 65535, then overflows).
/// - Compare Match A interrupt (OCIE1A) and Overflow interrupt (TOIE1) are enabled.
/// - Compare value (OCR1A = 24000) defines the timing point for Compare Match event.
/// - No prescaler is used (CS10 = 1), so timer runs at full CPU clock speed.
///
/// For example, with a 16 MHz clock:
///   - Each timer tick = 1 / 16,000,000 s = 62.5 ns
///   - OCR1A = 24000 → Compare Match every 24000 * 62.5 ns = 1.5 ms
///   - Full overflow (65536 ticks) = 4.096 ms
///
/// This setup can be used to generate PWM-like pulses or multiplexed servo control.
/// (Compare Match toggles output or triggers ISR to set pin LOW,
///  Overflow ISR can set pin HIGH for the next motor.)
void TimerControl::Timer1_Init() {
    TCNT1 = 0;                           // Reset timer counter
    OCR1A = 24000;                       // Set initial compare value (≈1.5 ms at 16 MHz)
    TIMSK1 = (1 << OCIE1A) | (1 << TOIE1); // Enable Compare A Match and Overflow interrupts
    TCCR1A = 0;                          // Normal mode (no PWM)
    TCCR1B = (1 << CS10);                // Prescaler = 1 → timer runs at full CPU speed
}

void TimerControl::setPinHigh(uint8_t port_pin_code) {
    uint8_t port = (port_pin_code >> 4) & 0x0F;
    uint8_t pin = port_pin_code & 0x0F;

    switch (port) {
    case 1: PORTA |= (1 << pin); break;
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
    case 1: PORTA &= ~(1 << pin); break;
    case 2: PORTB &= ~(1 << pin); break;
    case 3: PORTC &= ~(1 << pin); break;
    case 4: PORTD &= ~(1 << pin); break;
    default: break;
    }
}

void TimerControl::setup_Timers() {
    Timer1_Init();
    sei();
}

// ISRs must be outside the class, but can call static member functions or access static members
ISR(TIMER1_COMPA_vect) {	
    if (TimerControl::port_pin[TimerControl::curr_motor_i] > 0){
        TimerControl::setPinLow(TimerControl::port_pin[TimerControl::curr_motor_i]);
    }
}

ISR(TIMER1_OVF_vect) {
    TimerControl::curr_motor_i = (TimerControl::curr_motor_i +1) % 5;
    if (TimerControl::port_pin[TimerControl::curr_motor_i] > 0){
        TimerControl::setPinHigh(TimerControl::port_pin[TimerControl::curr_motor_i]);
        OCR1A = (uint16_t)(TimerControl::motors[TimerControl::curr_motor_i]*1600+24000)+112;
    }
}
