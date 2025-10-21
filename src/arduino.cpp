
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
	// Read the digital state of the pin
	if (pin <= 7) {
		return (PIND >> pin) & 1;
		} else if (pin >= 8 && pin <= 13) {
		return (PINB >> (pin - 8)) & 1;
	}
	// For other ports (e.g., C), you would add similar code.

	// Return a default value if an invalid pin is provided
	return 0;
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



uint8_t mapPinInnerRepresentation(uint8_t pin){
    
    //gets arduino pin number and gets port for that pin:
    //port: 1=A, 2=B, 3=C, 4=D
    uint8_t port = (pin < 8) ? 2 : 3; // default to port B
    switch(pin){
        case 0: port = 3; break; //C
        case 1: port = 3; break; //C
        case 2: port = 3; break; //C
        case 3: port = 3; break; //C
        case 4: port = 3; break; //C
        case 5: port = 3; break; //C
        case 6: port = 3; break; //C
        case 7: port = 3; break; //C
        case 8: port = 2; break; //B
        case 9: port = 2; break; //B
        case 10: port = 2; break; //B
        case 11: port = 2; break; //B
        case 12: port = 4; break; //D
        case 13: port = 4; break; //D
        case 14: port = 1; break; //A
        case 15: port = 1; break; //A
        default: return 255; //error
    }

    //gets pin number on that port (0-7)
    uint8_t pin_num = 0;
    switch(port){
        case 1:
            pin_num = pin;
            break;
        case 2:
            pin_num = pin - 8;
            break;
        case 3:
            pin_num = pin - 16;
            break;
        case 4:
            pin_num = pin - 24;
            break;
        default:
            return 255; //error
    }
    return (port << 4) | (pin_num & 0x0F);
};



void delay(double ms){
	_delay_ms(ms);
}

