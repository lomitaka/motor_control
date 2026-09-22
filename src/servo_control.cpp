#include "internals/timer_control.h"

#ifdef SIMULATION_MODE
    #include "simulator/avr_mock.h"
#else
    #include <avr/io.h>
    #include <avr/interrupt.h>
    #define __DELAY_BACKWARD_COMPATIBLE__
    #include "util/delay.h"
#endif
//#include <avr/io.h>
//#include <avr/interrupt.h>

#include "motor_control/servo_control.h"
#include "internals/arduino.h"

#include "stddef.h"

using namespace motor_control_internals;

volatile uint8_t ServoControl::curr_motor_i = 0;
volatile uint8_t ServoControl::serv_motor_count_ = 0;
volatile uint16_t ServoControl::serv_motors_[5] = {0, 0, 0, 0, 0};
volatile uint8_t ServoControl::serv_pin_[5] = {0, 0, 0, 0, 0};




/*
    //how it works (Servo approach)
    - Timer 1 is set to 4ms overflow period (With OCR1A) (16MHz / 1 prescaler / 64000 counts = 250 Hz → 4.000 ms)
    each cycle one servo motor is handled (5 motors max)
    on the beginning of the cycle the pin is set HIGH, and compare match B is set according to motor value
        when compare match occurs, pin is set LOW
    - thus each motor gets a pulse every 20ms (5 motors x 4ms)

    
*/


ServoControl::ServoControl(uint8_t pin) {

    if (!TimerControl::isInitialized()){
        TimerControl::setup_Timers();
    }
    port_pin_ = pin;
    motor_index = getServFreeMotorIndex(); //register motor

    if (motor_index < 0 ) return; //error, no free motor
    setServMotorPortPin(motor_index, port_pin_); //port B, pin 0
}


// Set target: for DC/stepper -> speed (-1000..1000), for servo -> angle (radians) depending on implementation
void ServoControl::setTarget(int16_t value){
    setServMotorValue(motor_index, value);
}

// Immediately set output (no ramp) — useful for calibration/emergency
void ServoControl::setImmediate(int16_t value){
    setServMotorValue(motor_index, value);
}





int8_t ServoControl::getServFreeMotorIndex(){
    for (int8_t i = 0; i < 5; i++) {
        if (serv_pin_[i] == 0) {
            return i;
        }
    }
    return -1; // No free motor index available
}

void ServoControl::freeServIndex(uint8_t index){
    /* goes over indexes, finds last non empty index, and clears it. 
    if no index found, then just clears item. */
    int8_t non_free_index = -1;
    for (int8_t i = serv_motor_count_; i <= 0 ; i--){
        if (index == i){continue;}
        if (serv_pin_[i] != 0){
            non_free_index =i;
        }
    }
    //there is another index, that is not used
    if (non_free_index >= 0){
        serv_pin_[index] = serv_pin_[non_free_index];
        serv_motors_[index] = serv_motors_[non_free_index];
    }else {
        serv_pin_[index] = 0;
        serv_motors_[index] = 0;
    }
    if (serv_motor_count_ > 0){
        serv_motor_count_--;
    }
}


void ServoControl::setServMotorValue(uint8_t index, int16_t value)
{
    //why 2000 and not 1000? probably because of different servo motors and ability
    // to go over edge, and not be limited, if there is some wierd servo implementation
    if (index < 5){
        if (value > 2000) value = 2000;
        if (value < -2000) value = -2000;
        cli();
        serv_motors_[index] = value;
        sei();
    }
}

    // Set port and pin for given motor index (0-4), pin: 1=1-13
    //set port to 0 to disable motor
void ServoControl::setServMotorPortPin(uint8_t index, uint8_t pin){
    if (index < 5 && pin <= 13){
        serv_pin_[index] = pin;
    }
}


void OnTimer1CompareMatchServo(){
    // store current motor index as a single char in dbg and terminate the string

    if (ServoControl::serv_pin_[ServoControl::curr_motor_i] > 0){
        //TimerControl::setPinLow(ServoControl::serv_pin_[TimerControl::curr_motor_i]);
        digitalWrite(ServoControl::serv_pin_[ServoControl::curr_motor_i],false);
    }
}

// 1 tick = 62.5 ns
// min = 1ms = 1 000 000 ns = 16000 ticks
// max = 2ms = 2 000 000 ns = 32000 ticks
//rescaling inpug value interval (-1000, 1000) to > (16000,32000)
void OnTimer1OwerflowServo(){
    //serv_dbg[0] = 'a';
   // DDRB |= (1 << (3)); PORTB |= (1 << (3));
    ServoControl::curr_motor_i = (ServoControl::curr_motor_i +1) % 5;
    
    if (ServoControl::serv_pin_[ServoControl::curr_motor_i] > 0){
        //TimerControl::setPinHigh(ServoControl::serv_pin_[TimerControl::curr_motor_i]);
        digitalWrite(ServoControl::serv_pin_[ServoControl::curr_motor_i],true);
        //maps value
        OCR1B = (uint16_t)((ServoControl::serv_motors_[ServoControl::curr_motor_i]+1000)*8+16000);
    }
}