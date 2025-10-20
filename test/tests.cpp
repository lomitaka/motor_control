/*
 * tests.cpp
 *
 * Created: 21.05.2024 18:49:32
 *  Author: Ivan
 */ 

/*Begining of Auto generated code by Atmel studio */

#include "headers/logic.h"
#include "headers/globalVariables.h"
#include "headers/usart.h"
#include "headers/timersControl.h"
#include "util/delay.h"
#include "headers/arduino.h"
#include "avr/io.h"
#include "Sketch.h"
#include "headers/CircularLogger.h"

#define BAUD 57600
#define MYUBRR round(FOSC/16.0/BAUD)-1



//valid settigns
//stty -F /dev/ttyACM0 57600 cs8 -parenb -cstopb
// Funkce pro čtení řádku
void readLine(char *buffer, uint8_t buffer_size) {
    //USART_WRITE_S("read line start\r\n");
	uint8_t index = 0;
    int received_char;

	while (1) {
		received_char = USART_PopReadBuffer(); // Přijmi znak
		if (received_char == -1){
					buffer[index] = '\0'; 
		return;
		}
		if (received_char == '\n' || received_char == '\r') { // Pokud je konec řádku
			buffer[index] = '\0'; // Ukonči řetězec
			//USART_WRITE_S("END");
			//USART_WRITE_S("read line break1\r\n"); // Debug zpráva
			//SART_Receive();
			//USART_WRITE_S("read line break2\r\n"); // Debug zpráva
			break;
		} else if (index < buffer_size - 1) { // Pokud je místo v bufferu
			buffer[index++] = received_char; // Přidej znak do bufferu
			//USART_WRITE_S("novy znak v bufferu: ") ;USART_WRITE_UINT((uint8_t)received_char);USART_WRITE_S("\r\n");
		} else {
			// Ochrana proti přetečení: ukonči řetězec
			buffer[index] = '\0';
			//USART_WRITE_S("buffer overflow\r\n");
			break;
		}
	}

	//USART_WRITE_S(buffer);
}



void servoExample(){
	ServoMotor servo1(3); //create servo motor object on pin 3
	servo1.configureRamp(500); //set ramp time to 500 ms
	servo1.setTarget(90); //set target angle to 90 degrees
	_delay_ms(3000); //wait for 3 seconds
	servo1.setTarget(0); //set target angle to 0 degrees
}

//samle usage of dc motor class
void dcExample1(){
	DCMotor motor1(9); //create DC motor object on pins 9 and 10 (PWM and direction)
	motor1.configureRamp(1000); //set ramp time to 1000 ms
	
	//reading number from serial line, and applying it as motor speed
	USART_Init()
	motor1.setTarget(speed);

	motor1.setTarget(500); //set target speed to 500 (out of -1000 to 1000)
	_delay_ms(5000); //wait for 5 seconds
	motor1.setTarget(0); //stop the motor
}


int main(void)
{
	//initialize system
	USART_Init(MYUBRR);
	sei(); //enable global interrupts

	//main loop
	while (1) 
	{
		loop();
	}
}