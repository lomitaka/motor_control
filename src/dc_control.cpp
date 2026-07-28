

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
#include "internals/fces.h"



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


/*

Implemenetd as:
    during each owerflow i will create sorted index array, that points to pins, pwm value array. 
    values are incremented ...todo.


*/

// Minimum time from current timer position before next compare event.
// Prevents missing compare match while inside ISR.
constexpr uint16_t TIMER_GUARD_TICKS = 100;


#define MAX_MOTOR_CNT 5
//currently used slot number (shows first empty index, max is 5 (by design))
volatile uint8_t DCControl::dc_motor_count_ = 0;

/// per class index to array _dc_motors/dc_port_dir_pin/dc_port/pwm_pin 
volatile uint8_t DCControl::dc_current_index_ = 0;

//ordered inexes of dc_motors_buffer
volatile uint8_t DCControl::dc_motors_off_order[5] = {0, 0, 0, 0, 0};

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
    port_dir_index_ = pin_direction;
    motor_index_ = getDCFreeMotorIndex(); //register motor

    if (motor_index_ < 0 ) {
        error_code_ = ErrorCodes::ERROR_NO_FREE_MOTOR;
        return error_code_;
    }; //error, no free motor
   setDCMotorPortPin(motor_index_, port_pwm_index_,port_dir_index_); //port B, pin 0
    
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

    if (value > 0){
        digitalWrite(DCControl::dc_port_dir_pin_[motor_index_],true);
    } else {
        digitalWrite(DCControl::dc_port_dir_pin_[motor_index_],false);
    }

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
    /* goes over indexes, finds last non empty index, and clears it. 
    if no index found, then just clears item. */
    int8_t non_free_index = -1;
    for (int8_t i = dc_motor_count_; i <= 0 ; i--){
        if (index == i){continue;}
        if (dc_port_pwm_pin_[i] != 0){
            non_free_index =i;
        }
    }
    //there is another index, that is not used
    if (non_free_index >= 0){
        dc_port_pwm_pin_[index] = dc_port_pwm_pin_[non_free_index];
        dc_port_dir_pin_[index] = dc_port_dir_pin_[non_free_index];
        dc_motors_[index] = dc_motors_[non_free_index];
    }else {
        dc_port_pwm_pin_[index] = 0;
        dc_port_dir_pin_[index] = 0;
        dc_motors_[index] = 0;
    }
    if (dc_motor_count_ > 0){
        dc_motor_count_--;
    }

    
}

void DCControl::setDCMotorValue(uint8_t index, int16_t value)
{
    if (index < 5){
        if (value > 1000) value = 1000;
        if (value < -1000) value = -1000;
        // dc_motors_ is 2 byte value, and there should be no way that this value will be updated only partially.
        cli();
        dc_motors_[index] = value;
        sei();
    }
}


    // Set port and pin for given motor index (0-4), port_pwm_pin, port_dir_pin are arduno pins.    
    //set port to 0 to disable motor
void DCControl::setDCMotorPortPin(uint8_t index, uint8_t port_pwm_pin, uint8_t port_dir_pin ){
    cli();    
        dc_port_pwm_pin_[index] = port_pwm_pin;
        dc_port_dir_pin_[index] = port_dir_pin;
    sei();
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


    bool mask[MAX_MOTOR_CNT] = {0,0,0,0,0};
    //for each of motors, find min pwm value, and put its index into dc_motors_off_order
    
    for (uint8_t i = 0;i < MAX_MOTOR_CNT;i++){
        int8_t min_index = -1; 

        for (int8_t j = 0;j < MAX_MOTOR_CNT;j++){
            if ((DCControl::dc_motors_buffer_[min_index] > DCControl::dc_motors_buffer_[j]) && !mask[j]){
                min_index = (int8_t)j;
            }
        }
        if (min_index >= 0){
            mask[min_index] = true;
            DCControl::dc_motors_off_order[i] = min_index;
        }
    }

    //set next OCR1B comparsion to min value.
    DCControl::dc_current_index_ = 0;
    if (DCControl::dc_port_dir_pin_[DCControl::dc_motors_off_order[0]] > 0){
        uint16_t minval =  DCControl::dc_motors_buffer_[DCControl::dc_motors_off_order[0]];
        //maps 0-1000 to 0-64000
        OCR1B = mabs(minval)*64; 
    }
   
    DCControl::dc_current_index_ = 1;
}

void OnTimer1CompareMatchDC(){
      
    digitalWrite(DCControl::dc_port_pwm_pin_[DCControl::dc_motors_off_order[DCControl::dc_current_index_]],false);
    
    uint16_t curr_value =  DCControl::dc_motors_buffer_[DCControl::dc_motors_off_order[0]];
    
    //sets new compare match value value for futrue
    //maps 0-1000 to 0-64000
    DCControl::dc_current_index_++;     
    OCR1B = max(TCNT1 + TIMER_GUARD_TICKS, mabs(curr_value)*64); 
}



