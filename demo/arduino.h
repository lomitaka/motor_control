
/*
 * arduino.h
 *
 * Created: 20.09.2023 20:51:03
 *  Author: Ivan
 */ 

// Define constants for pin modes


#include "stdint.h"

#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define LOW 0
#define HIGH 1

void pinMode(uint8_t pin, uint8_t mode);
int digitalRead(uint8_t pin);
void digitalWrite(uint8_t pin, uint8_t value);
uint16_t analogRead(uint8_t pin) ;
void delay(double ms);
