#include "internals/timer_control.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#define __DELAY_BACKWARD_COMPATIBLE__
#include "util/delay.h"
#include "motor_control/servo_control.h"
#include "internals/arduino.h"

#include "stddef.h"

volatile uint8_t ServoControl::serv_motor_count_ = 0;
volatile uint16_t ServoControl::serv_motors_[5] = {0, 0, 0, 0, 0};
volatile uint8_t ServoControl::serv_port_pin_[5] = {0, 0, 0, 0, 0};



ServoControl::ServoControl(uint8_t pin) {

    props.port_pin_ = mapPinInnerRepresentation(pin);
    props.motor_index = getServFreeMotorIndex(); //register motor

    if (props.motor_index < 0 ) return; //error, no free motor
    setServMotorPortPin(props.motor_index, props.port_pin_); //port B, pin 0
}

ServoControl::~ServoControl() {
    //releases motor index
    if (props.motor_index < 0 ) return; //error, no motor to free
    freeServIndex(props.motor_index);
}
// Set target: for DC/stepper -> speed (-1000..1000), for servo -> angle (radians) depending on implementation
void ServoControl::setTarget(int16_t value){
    setServMotorValue(props.motor_index, value);
}

// Immediately set output (no ramp) — useful for calibration/emergency
void ServoControl::setImmediate(int16_t value){
    setServMotorValue(props.motor_index, value);
}

// Configure ramping: type and time to reach target (milliseconds)
void ServoControl::configureRamp(int16_t rampTime_ms){
    props.ramp_.time_ms = rampTime_ms;
}






int8_t ServoControl::getServFreeMotorIndex(){
    for (int8_t i = 0; i < 5; i++) {
        if (serv_port_pin_[i] == 0) {
            return i;
        }
    }
    return -1; // No free motor index available
}

void ServoControl::freeServIndex(uint8_t index){
    serv_port_pin_[index] = 0;
    serv_motors_[index] = 0;
    
}


void ServoControl::setServMotorValue(uint8_t index, int16_t value)
{
    if (index < 5){
        if (value > 1000) value = 1000;
        if (value < -1000) value = -1000;
        serv_motors_[index] = value;
    }
}

    // Set port and pin for given motor index (0-4), port: 1=A, 2=B, 3=C, 4=D, pin: 0-7
    //set port to 0 to disable motor
void ServoControl::setServMotorPortPin(uint8_t index, uint8_t port, uint8_t pin){
    if (index < 5 && port <= 4 && pin <= 7){
        serv_port_pin_[index] = (port << 4) | (pin & 0x0F);
    }
}

    // Set port and pin for given motor index (0-4), port: 1=A, 2=B, 3=C, 4=D, pin: 0-7
    //set port to 0 to disable motor
void ServoControl::setServMotorPortPin(uint8_t index, uint8_t port_pin ){
    if (index < 5 && port_pin <= 0x4F){
        serv_port_pin_[index] = port_pin;
    }
}


void OnTimer1CompareMatchServo(){
    if (ServoControl::serv_port_pin_[TimerControl::curr_motor_i] > 0){
        TimerControl::setPinLow(ServoControl::serv_port_pin_[TimerControl::curr_motor_i]);
    }
}

// 1 tick = 62.5 ns
// min = 1ms = 1 000 000 ns = 16000 ticks
// max = 2ms = 2 000 000 ns = 32000 ticks
//rescaling inpug value interval (-1000, 1000) to > (16000,32000)
void OnTimer1OwerflowServo(){
    TimerControl::curr_motor_i = (TimerControl::curr_motor_i +1) % 5;
    if (ServoControl::serv_port_pin_[TimerControl::curr_motor_i] > 0){
        TimerControl::setPinHigh(ServoControl::serv_port_pin_[TimerControl::curr_motor_i]);
        OCR1A = (uint16_t)((ServoControl::serv_motors_[TimerControl::curr_motor_i]+1000)*8+16000);
    }
}