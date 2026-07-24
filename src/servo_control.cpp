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

volatile uint8_t ServoControl::serv_motor_count_ = 0;
volatile uint16_t ServoControl::serv_motors_[5] = {0, 0, 0, 0, 0};
volatile uint8_t ServoControl::serv_pin_[5] = {0, 0, 0, 0, 0};




ServoControl::ServoControl(uint8_t pin) {

    if (!TimerControl::isInitialized()){
        TimerControl::setup_Timers();
    }
    port_pin_ = pin;
    motor_index = getServFreeMotorIndex(); //register motor

    if (motor_index < 0 ) return; //error, no free motor
    setServMotorPortPin(motor_index, port_pin_); //port B, pin 0
}

ServoControl::~ServoControl() {
    //releases motor index
    //if (0 < 0 ) return; //error, no motor to free
    //freeServIndex(0);
}


// Set target: for DC/stepper -> speed (-1000..1000), for servo -> angle (radians) depending on implementation
void ServoControl::setTarget(int16_t value){
    setServMotorValue(motor_index, value);
}

// Immediately set output (no ramp) — useful for calibration/emergency
void ServoControl::setImmediate(int16_t value){
    setServMotorValue(motor_index, value);
}

// Configure ramping: type and time to reach target (milliseconds)
void ServoControl::configureRamp(int16_t rampTime_ms){
    //props.ramp_.time_ms = rampTime_ms;
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
    serv_pin_[index] = 0;
    serv_motors_[index] = 0;
    
}


void ServoControl::setServMotorValue(uint8_t index, int16_t value)
{
    //why 2000 and not 1000? probably because of different servo motors and ability
    // to go over edge, and not be limited, if there is some wierd servo implementation
    if (index < 5){
        if (value > 2000) value = 2000;
        if (value < -2000) value = -2000;
        serv_motors_[index] = value;
    }
}

    // Set port and pin for given motor index (0-4), pin: 1=1-13
    //set port to 0 to disable motor
void ServoControl::setServMotorPortPin(uint8_t index, uint8_t pin){
    if (index < 5 && pin <= 13){
        serv_pin_[index] = pin;
    }
}


/*
char serv_dbg[64];

// Přepíše dbg stringovou reprezentací TimerControl::curr_motor_i (0-255),
// maximálně 63 znaků + terminátor (dbg má velikost 64).
inline void PrintIfServPortPinIsOne(uint16_t v) {
    
    char *out = serv_dbg;
    // zvláštní případ 0
    if (v == 0) {
        out[0] = '0';
        out[1] = '\0';
        return;
    }
    // dočasné pole pro obrácené číslice (dostatečné pro uint32)
    char tmp[12];
    int tpos = 0;
    while (v > 0 && tpos < (int)sizeof(tmp) - 1) {
        tmp[tpos++] = (char)('0' + (v % 10));
        v /= 10;
    }
    // obrátíme a uložíme do dbg, ale nepřekročíme 63 znaků
    int maxlen = 63;
    int written = 0;
    while (tpos > 0 && written < maxlen) {
        out[written++] = tmp[--tpos];
    }
    out[written] = '\0';
}*/

void OnTimer1CompareMatchServo(){
    // store current motor index as a single char in dbg and terminate the string

    if (ServoControl::serv_pin_[TimerControl::curr_motor_i] > 0){
        //TimerControl::setPinLow(ServoControl::serv_pin_[TimerControl::curr_motor_i]);
        digitalWrite(ServoControl::serv_pin_[TimerControl::curr_motor_i],false);
    }
}

// 1 tick = 62.5 ns
// min = 1ms = 1 000 000 ns = 16000 ticks
// max = 2ms = 2 000 000 ns = 32000 ticks
//rescaling inpug value interval (-1000, 1000) to > (16000,32000)
void OnTimer1OwerflowServo(){
    //serv_dbg[0] = 'a';
   // DDRB |= (1 << (3)); PORTB |= (1 << (3));
    TimerControl::curr_motor_i = (TimerControl::curr_motor_i +1) % 5;
    if (ServoControl::serv_pin_[TimerControl::curr_motor_i] > 0){
        //TimerControl::setPinHigh(ServoControl::serv_pin_[TimerControl::curr_motor_i]);
        digitalWrite(ServoControl::serv_pin_[TimerControl::curr_motor_i],true);
        OCR1A = (uint16_t)((ServoControl::serv_motors_[TimerControl::curr_motor_i]+1000)*8+16000);
    }
}