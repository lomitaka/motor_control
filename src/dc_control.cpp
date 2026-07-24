

#ifdef SIMULATION_MODE
    #include "../simulator/avr_mock.h"
#else
    #include <avr/io.h>
    #include <avr/interrupt.h>
    #define __DELAY_BACKWARD_COMPATIBLE__
    #include "util/delay.h"
#endif

//#include <avr/io.h>
//#include <avr/interrupt.h>
//#define __DELAY_BACKWARD_COMPATIBLE__
//#include "util/delay.h"

#include "stddef.h"
#include "internals/timer_control.h"
#include "motor_control/dc_control.h"
#include "internals/arduino.h"



/* Main idea:
   - I have array of 5 motors, and i want to use them with Timer1 and Comparematch B register. 
   - so i will update COMPB register during cycle, always incementing it. 
   - each time, compare match occurs, i will find higher value to update to. if two values are same use index. 
   - on overflow set value to the lowest motor. 
*/ 

/*
    -Next Idea:
   have array of the values..  on the overflow coppy current values to the buffer. 
   -use values from buffer for next time cycle. (gives prolong over 4ms but i think it is ok)
   


*/

//currently used slot number (shows first empty index, max is 5 (by design))
volatile uint8_t DCControl::dc_motor_count_ = 0;
//current motor speed from -1000 to 1000
volatile uint16_t DCControl::dc_motors_[5] = {0, 0, 0, 0, 0};
//snapshot of ds_motors_ used for controll motor in a predictible way 
//this is updated once per owerflow
volatile uint16_t DCControl::dc_motors_buffer_[5] = {0, 0, 0, 0, 0};

/// @brief pins where pwm signal is generated
volatile uint8_t DCControl::dc_port_pwm_pin_[5] = {0, 0, 0, 0, 0};

/// @brief pins where direction is set
volatile uint8_t DCControl::dc_port_dir_pin_[5] = {0, 0, 0, 0, 0};


DCControl::DCControl(uint8_t pin_pwm,uint8_t pin_direction){
    init(pin_pwm,pin_direction);
}

uint8_t DCControl::init(uint8_t pin_pwm, uint8_t pin_direction) {

    port_pwm_index_ = pin_pwm;
    port_dir_index = pin_direction;
    motor_index_ = getDCFreeMotorIndex(); //register motor

    if (motor_index_ < 0 ) {
        error_code_ = ErrorCodes::ERROR_NO_FREE_MOTOR;
        return error_code_;
    }; //error, no free motor
   setDCMotorPortPin(motor_index_, port_pwm_index_,port_dir_index); //port B, pin 0
    
    return ErrorCodes::NO_ERROR;
}

void DCControl::setTarget(int16_t value){
    if (value > 1000) value = 1000;
    if (value < -1000) value = -1000;
    
    setDCMotorValue(motor_index_, value);
}

void DCControl::setImmediate(int16_t value){
    if (value > 1000) value = 1000;
    if (value < -1000) value = -1000;

    setDCMotorValue(motor_index_, value);
}

uint8_t DCControl::getLastError() {
    return error_code_;
};



int8_t DCControl::getDCFreeMotorIndex(){
    for (int8_t i = 0; i < 5; i++) {
        if (dc_port_pwm_pin_[i] == 0) {
            return i;
        }
    }
    return -1; // No free motor index available
}

void DCControl::freeDCIndex(uint8_t index){
    dc_port_pwm_pin_[index] = 0;
    dc_motors_[index] = 0;
    
}

void DCControl::setDCMotorValue(uint8_t index, int16_t value)
{
    if (index < 5){
        if (value > 1000) value = 1000;
        if (value < -1000) value = -1000;
        dc_motors_[index] = value;
    }
}

    // Set port and pin for given motor index (0-4), port: 1=A, 2=B, 3=C, 4=D, pin: 0-7
    //set port to 0 to disable motor
/*void DCControl::setDCMotorPortPin(uint8_t index, uint8_t port, uint8_t pin_pwm, uint8_t pin_dir){
    if (index < 5 && port <= 4 && pin_pwm <= 7){
        dc_port_pwm_pin_[index] = (port << 4) | (pin_pwm & 0x0F);
    }
}*/

    // Set port and pin for given motor index (0-4), port_pwm_pin, port_dir_pin are arduno pins.    
    //set port to 0 to disable motor
void DCControl::setDCMotorPortPin(uint8_t index, uint8_t port_pwm_pin, uint8_t port_dir_pin ){
        dc_port_pwm_pin_[index] = port_pwm_pin;
        dc_port_dir_pin_[index] = port_dir_pin;
}

/* 
    sets all enabled motors to high
    -sets Overflow to minimum of the values
*/
void OnTimer1OwerflowDC(){
    
    //copy each value to buffer, that is going to be applied for computations. 
    for (uint8_t i = 0;i < DCControl::dc_motor_count_;i++){
        if (DCControl::dc_port_pwm_pin_[i] > 0){
            digitalWrite(DCControl::dc_port_pwm_pin_[i],true);
            //update buffer
            DCControl::dc_motors_buffer_[i] = DCControl::dc_motors_[i];
        }
    }

    //finds minimal value of the 
    uint8_t minInd = 0;
    uint16_t minVal = 65535;
    for (uint8_t i = 0; i < DCControl::dc_motor_count_;i++){
        if (minVal > DCControl::dc_motors_buffer_[i]){
            minInd = i;
            minVal = DCControl::dc_motors_buffer_[i];
        }
    }
    //TODO compute right scaling
    OCR1B = (uint16_t)(DCControl::dc_motors_buffer_[minInd]*1600+24000)+112; 
    TimerControl::curr_dc_index = minInd;
}

bool isSmaller(uint8_t index1, uint8_t index2){
    
    
    if (DCControl::dc_motors_buffer_[index1] < DCControl::dc_motors_buffer_[index2]){
        return true;
    }
    if (DCControl::dc_motors_buffer_[index1] == DCControl::dc_motors_buffer_[index2]){
        return index1 < index2;
    }
    return false;
}

/* 
    check current motor value, and set 

*/
void OnTimer1CompareMatchDC(){
    
    //
    while (true){
        //set pin to low
        digitalWrite(DCControl::dc_port_pwm_pin_[TimerControl::curr_dc_index],false);
        //search for next motor to bring down
        int8_t nextIndex = -1;
        for (uint8_t i = 0; i < DCControl::dc_motor_count_;i++){
            
            //skips values, that were 
            if (DCControl::dc_motors_buffer_[i] < DCControl::dc_motors_buffer_[TimerControl::curr_dc_index] ||
                 i == TimerControl::curr_dc_index){ continue; }
            if (isSmaller(nextIndex, i)){
                nextIndex = i;
            }
        }

        
        if (nextIndex > 0){
            TimerControl::curr_dc_index = nextIndex;
            OCR1B = (uint16_t)(DCControl::dc_motors_[TimerControl::curr_dc_index]*1600+24000)+112; 

            //in case value already passed or is about to pass. (continue by next iteration)
            uint16_t timer1 = TCNT1;
            //1000 - TODO check if this value is reasonable (optionally to )
            if (OCR1B + 1000 > timer1 ){
                break;
            }
        }
    }
    
    
    
    
}



