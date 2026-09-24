#include "motor_control/dc_control.h"
#include "motor_control/dc_control_hbridge.h"
#include "arduino.h"
#include <avr/interrupt.h>
#include "fces.h"
#include "usart.h"

DCControl motor1(13);
DCControlHBridge motor2(12,11,10);



void ADC_Init()
{
    // Reference voltage = AVcc (typicky 5 V)
    // ADC input = ADC0 (kanál se bude měnit v analogRead)
    ADMUX = (1 << REFS0);

    // Enable ADC
    // Prescaler = 128
    // 16 MHz / 128 = 125 kHz ADC clock
    ADCSRA = (1 << ADEN)
           | (1 << ADPS2)
           | (1 << ADPS1)
           | (1 << ADPS0);

    // Optional: disable digital input buffers on ADC0-ADC5
    // DIDR0 = 0x3F;
}



#define MODE 6
#define SPEED 7
#define ACC 8
#define DIODE 9

static const uint16_t BUFFER_SIZE = 96;
static const unsigned int UART_BAUD = 57600;
static const unsigned int UART_UBRR = 16000000UL / 16 / UART_BAUD - 1;

int main() {
    //ADMUX = (ADMUX & 0x3F) | (1 << REFS0);
    //ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
   //USART_Init(UART_UBRR);
    sei();

    uint16_t speed_old = 0;  // 5 - safey reason ; //A1
    uint16_t acc_old = 0;

    sei();
    ADC_Init();
    while (true) {
        //chci. precist adc0.. pokud 01
        uint16_t mode = analogRead(14);//A0
        uint16_t speed = analogRead(15)/5;  // 5 - safey reason ; //A1
        uint16_t acc = analogRead(16);   //A2
        int16_t speed_sign = (int16_t)speed - 100;

        if (speed_sign < -100)
            speed_sign = -100;

        if (speed_sign > 100)
            speed_sign = 100;

        //USART_WRITE_S("MODE 1");
        //USART_WRITE_S("SPEED: "); USART_WRITE_UINT(speed);
        //USART_WRITE_S(" ACC: "); USART_WRITE_UINT(acc);
        //USART_WRITE_S("\n\r");

        if (mode < 460){
            digitalWrite(DIODE,true);
            //v0   //only one direction. 
            if (speed_old != speed){
                speed_sign = max((int16_t)0, speed_sign);
                motor1.setTarget((uint16_t)speed_sign);
                speed_old = speed;
            }
            if (acc_old != acc){
                //motor1.setAcceleration(acc);
                acc_old = acc;
            }
            

        } else if (mode < 540){
            //mid
            digitalWrite(DIODE,false);
        } else { 
            //v2
            if (speed_old != speed){
                motor2.setTarget(speed_sign);
                speed_old = speed_sign;
            }
            if (acc_old != acc){
                //motor2.setAcceleration(acc);
                acc_old = acc;
            }
            digitalWrite(DIODE,true);
        }

        delay(500);
    } 

}

    
