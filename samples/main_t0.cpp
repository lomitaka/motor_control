#include "motor_control/servo_control.h"
#include "motor_control/stepper_continuous.h"
#include "usart.h"
#include "arduino.h"


StepperContinuous sc;



#define FOSC 16000000 // Clock Speed
#define BAUD 57600
#define MYUBRR round(FOSC/16.0/BAUD)-1
#define BUFFER_SIZE 300

char buffer[BUFFER_SIZE];

uint16_t parse_num(const char *command){
	uint16_t result = 0;
	int i = 0;
	while (command[i] >= '0' && command[i] <= '9') {
			//USART_WRITE_UINT((uint8_t)c);
			result = result * 10 + (command[i] - '0');
			i++;
		}
	return result;
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


// Funkce pro analýzu příkazu
void processCommand(const char *command, StepperContinuous & sc) {
	if (command[0] == 'H') {
		USART_WRITE_S("T100\nA100\n");
		
	}

	//set speed
	if (command[0] == 'T'){
		// Příkaz SET - hledáme parametr a hodnotu
		uint8_t i = 0;                   // Pozice, kde začíná hodnota
		int8_t sign = 1;
		if (command[i] == '-') {
			i++; // Přeskoč znaménko mínus
			sign = -1;
		}

		int16_t speed = parse_num(command+ i+1);

		// Přečti hodnotu znaku po znaku

		speed = speed * sign;

		sc.setTargetSpeed(speed);
		USART_WRITE_S("speed set to"); USART_WRITE_UINT(speed); USART_WRITE_S("\r\n");
	}
	if (command[0] == 'A'){
		// Příkaz SET - hledáme parametr a hodnotu
		uint8_t i = 0;                   // Pozice, kde začíná hodnota
		int8_t sign = 1;
		if (command[i] == '-') {
			i++; // Přeskoč znaménko mínus
			sign = -1;
		}

		int16_t speed = parse_num(command+ i+1);

		// Přečti hodnotu znaku po znaku

		speed = speed * sign;
		sc.setAcceleration(speed);
		USART_WRITE_S("acceleration set to"); USART_WRITE_UINT(speed); USART_WRITE_S("\r\n");
	}
}



void setup(){

    sc.init(2,3);


	// Set reference voltage to AVcc with external capacitor at AREF pin
	ADMUX |= (1 << REFS0);
	ADMUX &= ~(1 << REFS1);
		
	// Enable the ADC and set the prescaler to max value (128)
	ADCSRA = 0b10000111;


    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    pinMode(14, INPUT);
    sc.setAcceleration(100);

	USART_Init(MYUBRR);
	sei(); //enable global interrupts
	
	//digitalWrite(2,HIGH);
	//delay(100);
	digitalWrite(3,LOW);
	//delay(100);

}



void loop() {
	
	for (int i = 0; i <200;i++){
		digitalWrite(2,HIGH);
		delay(5);
		digitalWrite(2,LOW);
		delay(5);
	}
	delay(2000);
	/*readLine(buffer, BUFFER_SIZE); // Načtení příkazu
	if (buffer[0] != '\0'){
		USART_WRITE_S("GOT CMD\r");
		delay(2);
		USART_WRITE_S(buffer);
		delay(2);
		processCommand(buffer, sc);       // Zpracování příkazu
	}*/


}//loop





int main(){				
	setup();
	while(true){
		loop();
	}
}