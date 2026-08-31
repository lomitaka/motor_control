#include "../include/motor_control/stepper_positioning.h"
#include "usart.h"
#include "arduino.h"
#include "../include/motor_control/debug.h"
#include "avr/interrupt.h"

StepperPositioning sp;



#define FOSC 16000000 // Clock Speed
#define BAUD 57600
#define MYUBRR round(FOSC/16.0/BAUD)-1
#define BUFFER_SIZE 300

char buffer[BUFFER_SIZE];

int16_t parse_num(const char *command){
	int16_t result = 0;
	
	uint8_t i = 0;                   // Pozice, kde začíná hodnota
	int8_t sign = 1;
	if (command[i] == '-') {
		i++; // Přeskoč znaménko mínus
		sign = -1;
	}
	
	while (command[i] >= '0' && command[i] <= '9') {
			//USART_WRITE_UINT((uint8_t)c);
			result = result * 10 + (command[i] - '0');
			i++;
		}
	return result*sign;
}



//valid settigns
//stty -F /dev/ttyACM0 57600 cs8 -parenb -cstopb
// Funkce pro čtení řádku
void readLine(char *buffer, uint16_t buffer_size) {
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

//heavily debugging functions
DebugInfo GetDebugInfo();


// Funkce pro analýzu příkazu
void processCommand(const char *command, StepperPositioning & sp) {
	if (command[0] == 'H') {
		USART_WRITE_S("T100 -tick to perform\nA100 - acceleration (empiric unit 0-100) \nS100 - speed");
		
	}

	//set speed
	if (command[0] == 't'){
		// Příkaz SET - hledáme parametr a hodnotu
		int16_t ticks_count = parse_num(command+ 1);

		// Přečti hodnotu znaku po znaku

		sp.setTargetTicks(ticks_count);
		USART_WRITE_S("set ticks to"); USART_WRITE_UINT(ticks_count); USART_WRITE_S("\r\n");
	}
	if (command[0] == 'a'){
		// Příkaz SET - hledáme parametr a hodnotu
		
		int16_t accel = parse_num(command+1);

		sp.setAcceleration(accel);
		USART_WRITE_S("acceleration set to"); USART_WRITE_UINT(accel); USART_WRITE_S("\r\n");
	}
	if (command[0] == 's'){
		// Příkaz SET - hledáme parametr a hodnotu
		

		uint16_t speed = (uint16_t)parse_num(command+ 1);

		// Přečti hodnotu znaku po znaku

		sp.setSpeed(speed);
		USART_WRITE_S("speed set to"); USART_WRITE_UINT(speed); USART_WRITE_S("\r\n");
	}

	if (command[0] == 'd'){
		for (int i = 0; i < 30;i++){
		DebugInfo dbg = GetDebugInfo();
		USART_WRITE_S("C:");USART_WRITE_LLONG(dbg.current_interval);USART_WRITE_S(":");
		USART_WRITE_S("R:");USART_WRITE_LLONG(dbg.remainining_interval);USART_WRITE_S(":");
		USART_WRITE_S("T:");USART_WRITE_LLONG(dbg.target_interval);;USART_WRITE_S("\r\n");
		delay(4);
		}
	}
}



void setup(){

    sp.init(2,3);


	// Set reference voltage to AVcc with external capacitor at AREF pin
	ADMUX |= (1 << REFS0);
	ADMUX &= ~(1 << REFS1);
		
	// Enable the ADC and set the prescaler to max value (128)
	ADCSRA = 0b10000111;


    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    pinMode(14, INPUT);
    sp.setAcceleration(10);

	USART_Init(MYUBRR);
	sei();
}



void loop() {
	

	readLine(buffer, BUFFER_SIZE); // Načtení příkazu
	if (buffer[0] != '\0'){
		USART_WRITE_S("GOT CMD\r");
		delay(2);
		USART_WRITE_S(buffer);
		delay(2);
		processCommand(buffer, sp);       // Zpracování příkazu
	}


}//loop





int main(){				
	setup();
	while(true){
		loop();
	}
}