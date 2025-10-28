/*Begining of Auto generated code by Atmel studio */

#include "test/internals/usart.h"
#include "util/delay.h"
#include "avr/io.h"
//#include "stddef.h"
#include <avr/interrupt.h>
#include <stdint.h>
#include <stddef.h>
#include "test/internals/arduino.h"

#include "motor_control/servo_control.h"
/* -------------- CONSTATNST */
#define FOSC 16000000 // Clock Speed
//#define BAUD 115200
//#define BAUD 9600
//#define BAUD 115200
#define BAUD 57600
#define MYUBRR round(FOSC/16.0/BAUD)-1



/*void setWheels(float left, float right){
	right = right*-1.0f; //right motor is mounted backward
	
	cli();
		currentMotorLeft = left+lCorrect ;// value in range from -1 to 1 
		currentMotorRight = right-rCorrect ;// value in range from -1 to 1 
	sei();
	lastRight = right;
	lastLeft =left;
	
}*/


#define BUFFER_SIZE 64 // Maximální délka příkazu
#define PARAM_MIN -1000    // Minimální hodnota parametru
#define PARAM_MAX 1000  // Maximální hodnota parametru

 int WH_LEFT=12;
 int WH_RIGHT=13;

//valid settigns
//stty -F /dev/ttyACM0 57600 cs8 -parenb -cstopb
// Funkce pro čtení řádku
void readLine(char *buffer, uint8_t buffer_size) {
	uint8_t index = 0;
    int received_char;

	while (1) {
		received_char = USART_Receive(); // Přijmi znak
		if (received_char == -1){
					//USART_WRITE_UINT(received_char);
					//USART_WRITE_S(" ");
					buffer[index] = '\0'; 
		return;
		}
		if (received_char == '\n' || received_char == '\r') { // Pokud je konec řádku
			//USART_WRITE_S("TERMINATING");
			buffer[index] = '\0'; // Ukonči řetězec
			break;
		} else if (index < buffer_size - 1) { // Pokud je místo v bufferu
			//USART_WRITE_S("ADDING TO BUFFER");
			buffer[index++] = received_char; // Přidej znak do bufferu
		} else {
			// Ochrana proti přetečení: ukonči řetězec
			buffer[index] = '\0';
			//USART_WRITE_S("buffer overflow\r\n");
			break;
		}
	}
}

ServoControl leftMotor(WH_LEFT);

// Funkce pro analýzu příkazu
void processCommand(const char *command) {
	if (command[0] == 'H') {
		USART_WRITE_S("S123 = 1.23  \n");
		USART_WRITE_S("CI123 = 1.23  \n");
		USART_WRITE_S("CB123 = 1.23  \n");
	}

	// Příkaz SET - hledáme parametr a hodnotu
	int16_t value = 0;               // Hodnota parametru
	uint8_t i = 0;                   // Pozice, kde začíná hodnota
	uint8_t sign = 1;
	if (command[i] == '-') {
		i++; // Přeskoč znaménko mínus
		sign = -1;
	}

	// Přečti hodnotu znaku po znaku
	while (command[i] >= '0' && command[i] <= '9') {
		//USART_WRITE_UINT((uint8_t)c);
		value = value * 10 + (command[i] - '0');
		i++;
	}
	value = value * sign;

	if (value >= PARAM_MIN && value <= PARAM_MAX) {
		// Nastavení parametru
		leftMotor.setTarget(value);
		USART_WRITE_S("A set to"); USART_WRITE_FLOAT(value); USART_WRITE_S("\r\n");
		
	} else {
		USART_WRITE_S("Error: Value out of range (-1000-1000).\n");
	}
    
}




void setup() {
	
		
	pinMode(WH_LEFT,OUTPUT);
	pinMode(WH_RIGHT,OUTPUT);

	USART_Init(MYUBRR);

	
	//lastCallTime = millis();
}


bool log_written = false;
size_t start_time;


char buffer[BUFFER_SIZE];

void loop() {

	//lastCallTime = millis();	
	
	//
	
	readLine(buffer, BUFFER_SIZE); // Načtení příkazu
	if (buffer[0] != '\0'){
		USART_WRITE_S("GOT CMD\r");
		delay(2);
		USART_WRITE_S(buffer);
		delay(2);
		processCommand(buffer);       // Zpracování příkazu
	}
	

}//loop

int main(){				
	setup();
	USART_WRITE_S("SERIAL ONLINE:\r\n");
	while(true){
		loop();
	}
}