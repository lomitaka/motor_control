#include "../include/motor_control/stepper_continuous.h"
#include "usart.h"
#include "arduino.h"
#include <avr/interrupt.h>

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



struct Interval {
    uint8_t skip_count;//255 for full stop
    uint16_t remainder;

    // Elegantnější cesta: Přetížení operátoru <
    // Pozor: Operátor v C++ standardně bere jen pravou stranu, 
    // takže divider musíme předat jinak, nebo předpokládat fixní divider.
    // Zde ukázka s předpokladem stejného divideru pro obě struktury:
    bool operator<(const volatile Interval& other) const volatile {
        uint8_t sc1 = skip_count;
        uint8_t sc2 = other.skip_count;
        if (sc1 != sc2) return sc1 < sc2;
        
        uint16_t rem1 = remainder;
        uint16_t rem2 = other.remainder;
        return rem1 < rem2;
    }

    // Přetížení operátoru rovnosti (==)
    bool operator==(const volatile Interval& other) const volatile {
        return (skip_count == other.skip_count) && (remainder == other.remainder);
    }

    // Přetížení operátoru nerovnosti (!=)
    bool operator!=(const volatile Interval& other) const volatile {
        return (skip_count !=  other.skip_count) || (remainder != other.remainder);
    }
    
    bool operator>(const volatile Interval& other) const volatile {
    uint8_t sc1 = skip_count;
    uint8_t sc2 = other.skip_count;
    if (sc1 != sc2) return sc1 > sc2; // Větší než (>)
    
    uint16_t rem1 = remainder;
    uint16_t rem2 = other.remainder;
    return rem1 > rem2; // Větší než (>)
    }

    volatile Interval& operator=(const volatile Interval& other) volatile {
        // Bezpečně načteme hodnoty z 'other' do lokálních proměnných 
        // (pokud by 'other' byl náhodou také volatile)
        uint8_t sc = other.skip_count;
        uint16_t rem = other.remainder;

        // Zapíšeme je do našich volatile položek
        skip_count = sc;
        remainder = rem;

        // Operátor přiřazení v C++ standardně vrací referenci na sebe, 
        // abyste mohli řetězit přiřazení typu a = b = c;
        return *this;
    }

};



#define STOP 250
#define SLOW 9
#define VERY_SLOW 244
#define VERY_SLOW2 (uint32_t)244 << 16


void addToInterval2a(volatile Interval & target, volatile Interval & new_current, uint8_t modifier){
    //modifier -number from 1-10 for more aggresive interval update

    if (target > new_current){
        //need to add to new_current
        uint32_t new_curr = ((uint32_t)new_current.skip_count << 16) + new_current.remainder;
        //add 8 percent of new_current to new_curr
        new_curr = new_curr + modifier*(new_curr >> 3);
        //handle top (converts it into stop)target_speeds_stp_ps_
        if (new_current.skip_count > VERY_SLOW ) {new_curr = ((uint32_t)VERY_SLOW << 16);}
        new_current.remainder = new_curr & 0xFFFF;
        new_current.skip_count = new_curr >> 16;
        
        if (new_current > target){
            new_current.remainder = target.remainder;
            new_current.skip_count = target.skip_count;
        }
        
    }else if (target < new_current){
        uint32_t new_curr = ((uint32_t)new_current.skip_count << 16) + new_current.remainder;
        //add 8 percent of new_current to new_curr
        
        new_curr = new_curr - modifier*(new_curr >> 3);
        //lower protectin
        if (new_curr < 200) {new_curr = 200;}
        new_current.remainder = new_curr & 0xFFFF;
        new_current.skip_count = new_curr >> 16;
        
        if (new_current < target){
            new_current.remainder = target.remainder;
            new_current.skip_count = target.skip_count;
        }
    }
}



void addTest(){

    uint32_t targetn =  589824;
    uint32_t currentn =  13421774;
    volatile Interval target;
    target.skip_count = targetn / 65536;
    target.remainder = targetn % 65536;
    
    volatile Interval current;
    current.skip_count = currentn / 65536;
    current.remainder = currentn % 65536;

	USART_WRITE_S("\n\r");
	USART_WRITE_S("\n\r");
    for (int i = 0 ; i < 10; i++){

        addToInterval2a(target, current, 1);
        uint32_t val =  ((uint32_t)current.skip_count << 16)+ current.remainder;
        USART_WRITE_LLONG(val);
		USART_WRITE_S("\n\r");
    }
}





void loop() {
	
	
	addTest();
	
	delay(20000);
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