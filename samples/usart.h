/* 
* usart.h
*
* Created: 24.08.2023 14:21:58
* Author: Ivan
*/


#ifndef __USART_H__
#define __USART_H__


//#include <Arduino.h>

void USART_TransmitCND(unsigned char data);
void USART_WRITE_S(const char * s);

void USART_WRITE_UINT(unsigned long number);
void USART_WRITE_INT(int number);
void USART_WRITE_FLOAT(float number);

void USART_Transmit(unsigned char data);
void USART_TransmitCND(unsigned char data);
unsigned char USART_Receive(void);
unsigned char USART_Receive_Async(void);
unsigned char USART_DataReadAvailable(void);
void USART_Init(unsigned int ubrr);



void appendLog(const char* data);


int USART_PopReadBuffer() ;
void USART_Discard_ReadBuffer();


#endif //__USART_H__
