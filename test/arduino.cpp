
/*
* arduino.cpp
*
* Created: 20.09.2023 20:49:54
*  Author: Chat GPT
*/

#include <avr/io.h>
#define __DELAY_BACKWARD_COMPATIBLE__
#include "util/delay.h"


// Define constants for pin modes
#define INPUT 0
#define OUTPUT 1

// Define constants for pin modes
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define HIGH 1
#define LOW 0

void pinMode(uint8_t pin, uint8_t mode) {
	if (mode == OUTPUT) {
		// Set the Data Direction Register (DDRx) bit for the pin to configure it as an output
		if (pin <= 7) {
			DDRD |= (1 << pin);
			} else if (pin >= 8 && pin <= 13) {
			DDRB |= (1 << (pin - 8));
			} else if (pin >= 14 && pin <= 19) {  // Pins A0-A5 are typically 14-19 on Arduino Uno
			DDRC |= (1 << (pin - 14));
		}
		} else if (mode == INPUT) {
		// Clear the Data Direction Register (DDRx) bit for the pin to configure it as an input
		if (pin <= 7) {
			DDRD &= ~(1 << pin);
			PORTD &= ~(1 << pin);  // Disable internal pull-up resistor
			} else if (pin >= 8 && pin <= 13) {
			DDRB &= ~(1 << (pin - 8));
			PORTB &= ~(1 << (pin - 8));  // Disable internal pull-up resistor
			} else if (pin >= 14 && pin <= 19) {
			DDRC &= ~(1 << (pin - 14));
			//PORTC &= ~(1 << (pin - 14));  // Disable internal pull-up resistor
		}
		} else if (mode == INPUT_PULLUP) {
		// Clear the Data Direction Register (DDRx) bit for the pin to configure it as an input
		if (pin <= 7) {
			DDRD &= ~(1 << pin);
			PORTD |= (1 << pin);  // Enable internal pull-up resistor
			} else if (pin >= 8 && pin <= 13) {
			DDRB &= ~(1 << (pin - 8));
			PORTB |= (1 << (pin - 8));  // Enable internal pull-up resistor
			} else if (pin >= 14 && pin <= 19) {
			DDRC &= ~(1 << (pin - 14));
			PORTC |= (1 << (pin - 14));  // Enable internal pull-up resistor
		}
	}
}


void digitalWrite(uint8_t pin, uint8_t value) {
	if (value == HIGH) {
		// Set the pin to HIGH
		if (pin <= 7) {
			PORTD |= (1 << pin);
			} else if (pin >= 8 && pin <= 13) {
			PORTB |= (1 << (pin - 8));
		}
		// For other ports (e.g., C), you would add similar code.
		} else if (value == LOW) {
		// Set the pin to LOW
		if (pin <= 7) {
			PORTD &= ~(1 << pin);
			} else if (pin >= 8 && pin <= 13) {
			PORTB &= ~(1 << (pin - 8));
		}
		// For other ports (e.g., C), you would add similar code.
	}
}

/**/
int digitalRead(uint8_t pin) {
	if (pin <= 7) {
		return (PIND >> pin) & 1;
	} else if (pin >= 8 && pin <= 13) {
		return (PINB >> (pin - 8)) & 1;
	} else if (pin >= 14 && pin <= 19) { // A0-A5
		return (PINC >> (pin - 14)) & 1;
	}
	// Invalid pin -> return LOW
	return LOW;
}




/* returns 10bit number */
uint16_t analogRead(uint8_t pin) {
	// Check if the pin is within the valid range for analog inputs (A0-A5)
	if (pin < 14 || pin > 19) {
		return 0; // Invalid pin, return 0
	}

	// Map Arduino pin numbers to ADC channels (A0-A5 -> ADC0-ADC5)
	uint8_t adcChannel = pin - 14;

	// Select the corresponding ADC channel
	ADMUX = (ADMUX & 0xF0) | adcChannel;

	ADCSRA |= (1 << ADSC);

	// Wait for conversion to complete
	while (ADCSRA & (1 << ADSC));

	// Read ADC value
	uint16_t result = ADCW;

	// Disable ADC to save power
	//ADCSRA &= ~(1 << ADEN);

	return result;
}





void delay(double ms){
	_delay_ms(ms);
}

