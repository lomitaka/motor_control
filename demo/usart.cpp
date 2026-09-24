/* 
* usart.cpp
*
* Created: 24.08.2023 14:21:58
* Author: Ivan
*/


#include "usart.h"
#define __DELAY_BACKWARD_COMPATIBLE__
#include "util/delay.h"
#include "avr/io.h"
#include "string.h"
#include "stdio.h"
#include <avr/interrupt.h>

#define BUFFER_SIZE 300

char transmitBuffer[BUFFER_SIZE];  // Circular buffer for storing data to be sent
volatile uint8_t bufferHead = 0;   // Points to the position where new data is added
volatile uint8_t bufferTail = 0;   // Points to the next character to be sent
volatile bool transmitting = false; // True when transmission is in progress

void USART_Init(unsigned int ubrr)
{
	/*Set baud rate */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;

	//Enable receiver and transmitter
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)| (1<<UDRIE0);//  | (1 << RXCIE0);
	/* Set frame format: 8data, 1stop bit */
	UCSR0C = (3<<UCSZ00);
	/*UCSR0C = 0x06;*/
}



unsigned char USART_Receive_Async(void)
{
	if ((UCSR0A & (1<<RXC0))){
		/* Get and return received data from buffer */
		return UDR0;
	}
	else {
		return 0;
	}
}

unsigned char USART_DataReadAvailable(void)
{
	return (UCSR0A & (1<<RXC0));
}

unsigned char USART_Receive(void)
{
	//UCS
	/* Wait for data to be received */
	while (!(UCSR0A & (1<<RXC0)))
	;
	return UDR0;
	/* Get and return received data from buffer */
	//unsigned char  data = UDR0 ;//& ~0x80;//some wierd bug, that all data received starts with 1
	//return data; 
}

void USART_TransmitCND(unsigned char data){
	USART_Transmit(data);
}


void USART_WRITE_S(const char * s){
	#ifdef SIMULATION
	USART_TransmitCND('s');
	USART_TransmitCND(strlen(s));
	#endif
	cli();
	for (uint16_t i =0; i < strlen(s);i++){
		USART_TransmitCND(s[i]);
	}
	sei();
}

void USART_WRITE_UINT(unsigned long number){
	char str[20];
	sprintf(str, "%lu", number);
	USART_WRITE_S(str);
}
void USART_WRITE_INT(int number){
	char str[20]{32,32,32,32,32,32,32,32,32,65};
	sprintf(str, "%d", number);
	USART_WRITE_S(str);
}
void USART_WRITE_LLONG(long long number){
	if (number < 0) {
		USART_WRITE_S("-");	
		number = -number;
	}

	if (number > 4294967296){
		uint32_t base = number / 4294967296;
		USART_WRITE_UINT(base);
		number = number % 4294967296;

	}

	uint32_t number32 = (uint32_t)number;
	char str[22]{32,32,32,32,32,32,32,32,32,32,32,65};
	sprintf(str, "%ld", number32);
	USART_WRITE_S(str);
}
void USART_WRITE_FLOAT(float number){
	float n2 = number;
	if (n2 < 0){
		n2 = -n2;
		USART_WRITE_S("-");
	}
	
	if (n2 >= 1){
		USART_WRITE_INT(floor(n2));
		USART_WRITE_S(".");
		n2 = n2 - floor((n2));
	} else if (n2 < 1 && n2 >= 0) {
		USART_WRITE_S("0.");
	}
	
	USART_WRITE_INT((int)(floor(n2*1000)));
	
}

void USART_Transmit(unsigned char data)
{
	// Wait for empty transmit buffer 
	while (!(UCSR0A & (1<<UDRE0)))
	;
	// Put data into buffer, sends the data 
	UDR0 = data;
	_delay_ms(1.0f);
}


void appendLog(const char* data) {
	// Add data to the buffer until it's full or until the string ends
	while (*data) {
		uint8_t nextHead = (bufferHead + 1) % BUFFER_SIZE;
		// Check if the buffer is full
		if (nextHead == bufferTail) {
			// Buffer is full, wait or discard the character
			break;
		}
		transmitBuffer[bufferHead] = *data++;
		bufferHead = nextHead;  // Move the head forward
	}
	// Start transmission if not already transmitting
	if (!transmitting) {
		transmitting = true;
		// Trigger transmission by enabling UDRE interrupt
		UCSR0B |= (1 << UDRIE0);
	}
}


// Interrupt service routine for UART data register empty
ISR(USART_UDRE_vect) {
	// Check if there's data to send
	if (bufferHead != bufferTail) {
		// Load the next byte from the buffer
		UDR0 = transmitBuffer[bufferTail];
		bufferTail = (bufferTail + 1) % BUFFER_SIZE;  // Move the tail forward
		} else {
		// Buffer is empty, disable UDRE interrupt
		UCSR0B &= ~(1 << UDRIE0);
		transmitting = false;
	}
}



#define BUFFER_SIZE2 128  // Velikost kruhového bufferu

// receive buffer from serial line
volatile uint8_t rxBuffer[BUFFER_SIZE2];
volatile uint8_t head = 0; // Index pro zápis do bufferu
volatile uint8_t tail = 0; // Index pro čtení z bufferu


// ISR pro příjem dat
ISR(USART_RX_vect) {
    uint8_t data = UDR0;  // Načtení přijatého bajtu
    uint8_t nextHead = (head + 1) % BUFFER_SIZE2; // Další pozice v bufferu

    // Pokud buffer není plný, uložit data
    if (nextHead != tail) {
        rxBuffer[head] = data;
        head = nextHead; // Posunout ukazatel na další volné místo
    }
    // Jinak je buffer plný, data se ztratí (možno přidat obsluhu přetečení)
}


// Funkce pro čtení z bufferu
int USART_PopReadBuffer() {
    if (head == tail) {
        return -1; // Buffer je prázdný
    } else {
        uint8_t data = rxBuffer[tail];
        tail = (tail + 1) % BUFFER_SIZE2; // Posunout ukazatel čtení
        return data;
    }
}

void USART_Discard_ReadBuffer(){
	cli();
	head = tail = 0;
	sei();
}