


#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define HIGH 1
#define LOW 0


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

void pinMode(uint8_t pin, uint8_t mode) ;


/* returns 10bit number */
uint16_t analogRead(uint8_t pin) ;

void digitalWrite(uint8_t pin, uint8_t value);

void delay(double ms);


void readLine(char *buffer, uint16_t buffer_size);
