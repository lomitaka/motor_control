/*
 * timersControl.cpp
 *
 * Created: 18.09.2023 21:40:23
 *  Author: Ivan
 */ 

#include "headers/globalVariables.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#define __DELAY_BACKWARD_COMPATIBLE__
#include "util/delay.h"
#include "headers/usart.h"
#include <avr/io.h>
#include "stddef.h"
#include "headers/logic.h"
/*

Notes:
frequency 32mhz



timer 0:
	use it to measure milisecconds since start. 2ms tick
	    prescaler 256
		ocr 249

timer 1: set timer to from 0 to max, in time cca 4ms. 
		on timer overflow set new ocr for given motor.  set motor HIGH
		on compare match, pull motor LOW.  
		
		motors are multiplexed. in cycle i, works motor i. (max 5 motors allowd. )


timer 2: use it to measure ticks of length 20us. 
		to decrease left_cntr and right_cntr, and check if low signal should be already set
		on overflow stops timer. (in normal mode)
		32 prescaler valid range should be tween 0 and 500us


Here is your description translated into English:

---

**Principle**

Timer1 runs continuously from 0 upwards (to 65535).

When it overflows (OVF):

- It switches to the next motor (`curr_motor_i`),
- Sets the pin for that motor to HIGH,
- At the same time, calculates how long the pin should stay HIGH → this value is stored in `OCR1A`.

When Timer1 reaches `OCR1A` (Compare Match A):

- The motor pin is set to LOW.

In this way, each motor receives a short HIGH pulse, which carries information about its speed/angle.
At any given moment, only one motor is active, but because they are switched rapidly, all motors appear to be running simultaneously.



*/

volatile size_t timeSinceStart = 0;

//uses to indicate which motor currently receives signal.
volatile uint8_t curr_motor_i = 0;
volatile uint8_t motor_count = 0;
volatile float motors[5] = {0.0,0.0,0.0,0.0,0.0};/* value in range from -1 to 1 */
volatile uint8_t port_pin[5] = {0x00, 0x00, 0x00, 0x00, 0x00};/* port and pin assignments for each motor higher 4 bytes is  */
// port number A = 1, B = 2, C = 3, D = 4, lower 4 bytes is pin number 0-7

void Timer0_Init() {//used to detect time

	// Set Timer0 set to CTC mode
	TCCR0A = (1 << WGM01);
	uint8_t target = 124;
	OCR0A = target;
	//OCR0A = 0x7C;// target;
	
	// Formula: Prescaler = (ClockFrequency / (DesiredTime * 256)) - 1
	TCCR0B =  (1 << CS02); //sets prescaler to 256
	//TCCR0B = 0b00000101;//1024
	TIMSK0 = (1 << OCIE0A);
}


void Timer1_Init() {
    // Nastavení počáteční hodnoty časovače na 0
    TCNT1 = 0;

    // Nastavení registru OCR1A na výchozí hodnotu
    OCR1A = 24000;

    // Povolte přerušení při Compare Match A a Overflow
    TIMSK1 = (1 << OCIE1A) | (1 << TOIE1);

    // Režim "Normal Mode" (časovač běží od 0 do 65535 bez nulování při OCR1A)
    TCCR1A = 0;
    TCCR1B = (1 << CS10); // Prescaler = 1 (časovač běží na plnou frekvenci)
}


void setPinHigh(uint8_t port_pin_code) {
	uint8_t port = (port_pin_code >> 4) & 0x0F;
	uint8_t pin = port_pin_code & 0x0F;

	switch (port) {
		case 1: // Port A
			PORTA |= (1 << pin);
			break;
		case 2: // Port B
			PORTB |= (1 << pin);
			break;
		case 3: // Port C
			PORTC |= (1 << pin);
			break;
		case 4: // Port D
			PORTD |= (1 << pin);
			break;
		default:
			break;
	}
}

void setPinLow(uint8_t port_pin_code) {
	uint8_t port = (port_pin_code >> 4) & 0x0F;
	uint8_t pin = port_pin_code & 0x0F;

	switch (port) {
		case 1: // Port A
			PORTA &= ~(1 << pin);
			break;
		case 2: // Port B
			PORTB &= ~(1 << pin);
			break;
		case 3: // Port C
			PORTC &= ~(1 << pin);
			break;
		case 4: // Port D
			PORTD &= ~(1 << pin);
			break;
		default:
			break;
	}
}

void Timer2_Init() {

	//TCCR2A = //normal operation mode
	//uint8_t target = 9;//should create tick each 20us.
	uint8_t target = 19;//should create tick each 10us.
	OCR2A = target;
	// Set the prescaler to 32
	//TCCR2B |= (1 << CS21)| (1 << CS20);
	
	// Set the prescaler to 8
	TCCR2B |= (1 << CS21);

	//enable interrupts of timer 2
	TIMSK2 = (1 << OCIE2A);
 	TCCR2A = (1 << WGM01);
}



ISR(TIMER1_COMPA_vect) {	
	//if motor pin is set, set it to LOW
	if (port_pin[curr_motor_i] > 0){
		setPinLow(port_pin[curr_motor_i]);
	}
}


// ISR pro přerušení při přetečení časovače
ISR(TIMER1_OVF_vect) {
	curr_motor_i = (curr_motor_i +1) %5;
    
	//set motor to HIGH
	if (port_pin[curr_motor_i] > 0){
		//PORTB |= (1 << PINB4);
		setPinHigh(port_pin[curr_motor_i]);
		
		//tick delay is 0,0625us  so 1500us = 24000
		//1400us = 27200- 1600
		//1600us = 27200+1600
		//left_cntr min=1400us = 140  middle=150 max = 160  (1600)
		OCR1A = (uint16_t)(motors[curr_motor_i]*1600+24000)+112;//correction. DO now know why is this
	}
}

ISR(TIMER0_COMPA_vect) {
	//measures milliseconds.
	timeSinceStart+=2;
}



void setup_Timers(){
	
	Timer0_Init();
	
	
	Timer1_Init();

	
	//Timer2_Init();

	// Enable global interrupts
	sei();
}


size_t millis(){
	
	cli();
	size_t result = timeSinceStart;
	sei();
	return result;
}
