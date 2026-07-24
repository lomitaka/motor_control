// Mock AVR hlavičkový soubor - nahrazuje <avr/io.h> a <avr/interrupt.h> pro simulaci
#ifndef AVR_MOCK_H
#define AVR_MOCK_H

#include <stdint.h>
#include <functional>

// Forward deklarace namespace
namespace AVRSim {
    extern volatile uint16_t TCNT1;
    extern volatile uint16_t OCR1A;
    extern volatile uint16_t OCR1B;
    extern volatile uint8_t TCCR1A;
    extern volatile uint8_t TCCR1B;
    extern volatile uint8_t TIMSK1;
    extern volatile uint8_t TIFR1;
    
    extern volatile uint8_t PORTB;
    extern volatile uint8_t PORTC;
    extern volatile uint8_t PORTD;
    extern volatile uint8_t DDRB;
    extern volatile uint8_t DDRC;
    extern volatile uint8_t DDRD;
    extern volatile uint8_t PINB;
    extern volatile uint8_t PINC;
    extern volatile uint8_t PIND;
    
    extern volatile uint8_t SREG;
    
    extern const uint8_t CS10;
    extern const uint8_t CS11;
    extern const uint8_t CS12;
    extern const uint8_t OCIE1A;
    extern const uint8_t OCIE1B;
    extern const uint8_t TOIE1;
    extern const uint8_t TOV1;
    extern const uint8_t OCF1A;
    extern const uint8_t OCF1B;
}

// Použití registrů bez префикsu
using AVRSim::TCNT1;
using AVRSim::OCR1A;
using AVRSim::OCR1B;
using AVRSim::TCCR1A;
using AVRSim::TCCR1B;
using AVRSim::TIMSK1;
using AVRSim::TIFR1;

using AVRSim::PORTB;
using AVRSim::PORTC;
using AVRSim::PORTD;
using AVRSim::DDRB;
using AVRSim::DDRC;
using AVRSim::DDRD;
using AVRSim::PINB;
using AVRSim::PINC;
using AVRSim::PIND;

using AVRSim::SREG;

using AVRSim::CS10;
using AVRSim::CS11;
using AVRSim::CS12;
using AVRSim::OCIE1A;
using AVRSim::OCIE1B;
using AVRSim::TOIE1;
using AVRSim::TOV1;
using AVRSim::OCF1A;
using AVRSim::OCF1B;

// Makra pro interrupt enable/disable
#define sei() AVRSim::SREG |= 0x80
#define cli() AVRSim::SREG &= ~0x80

// Mock delay funkce (neprovádí skutečné čekání)
#define _delay_ms(x) /* no operation in simulation */
#define _delay_us(x) /* no operation in simulation */

// F_CPU pro kompatibilitu
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

// ISR makro pro simulaci - funkce se definují normálně
#define ISR(vector) void vector##_func()

// Vektory přerušení
#define TIMER1_COMPA_vect TIMER1_COMPA
#define TIMER1_OVF_vect TIMER1_OVF

#endif // AVR_MOCK_H
