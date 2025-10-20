#pragma once
#include <stdint.h>
#include "headers/timer_control.h"
#include "headers/ramp.h"

//enum class RampType { None, Linear, SCurve };

namespace ErrorCodes {
    enum ErrorCode {
        NO_ERROR = 0,
        ERROR_NO_FREE_MOTOR = -1,
        ERROR_INVALID_PIN = -2,
        ERROR_INVALID_VALUE = -3
};
}

// basic state reported by motor
struct MotorState {
    int16_t currentSetpoint; // -1000..1000 for speed, or radians for servo depending on mode
    int16_t targetSetpoint;
    bool running;
};

class IMotor {
public:

    virtual ~IMotor() {};

    // Initialize motor with a specific value
    //returns error code if failed or 0 if success
    virtual uint8_t init(uint8_t pin) = 0;

    // Set target: for DC/stepper -> speed (-1000..1000), for servo -> angle (degrees) depending on implementation
    virtual void setTarget(int16_t value) = 0;
    
    // Immediately set output (no ramp) — useful for calibration/emergency
    virtual void setImmediate(int16_t value) = 0;

    // Configure ramping: type and time to reach target (milliseconds)
    virtual void configureRamp(int16_t rampTime_ms) = 0;


protected:
/* Maps arduino pin number to  port| port_pin*/
uint8_t mapPinInnerRepresentation(uint8_t pin){
    
    //gets arduino pin number and gets port for that pin:
    //port: 1=A, 2=B, 3=C, 4=D
    uint8_t port = (pin < 8) ? 2 : 3; // default to port B
    switch(pin){
        case 0: port = 3; break; //C
        case 1: port = 3; break; //C
        case 2: port = 3; break; //C
        case 3: port = 3; break; //C
        case 4: port = 3; break; //C
        case 5: port = 3; break; //C
        case 6: port = 3; break; //C
        case 7: port = 3; break; //C
        case 8: port = 2; break; //B
        case 9: port = 2; break; //B
        case 10: port = 2; break; //B
        case 11: port = 2; break; //B
        case 12: port = 4; break; //D
        case 13: port = 4; break; //D
        case 14: port = 1; break; //A
        case 15: port = 1; break; //A
        default: return 255; //error
    }

    //gets pin number on that port (0-7)
    uint8_t pin_num = 0;
    switch(port){
        case 1:
            pin_num = pin;
            break;
        case 2:
            pin_num = pin - 8;
            break;
        case 3:
            pin_num = pin - 16;
            break;
        case 4:
            pin_num = pin - 24;
            break;
        default:
            return 255; //error
    }
    return (port << 4) | (pin_num & 0x0F);
};

private:
    uint8_t error_code_ = 0;
};

class DCMotor : public IMotor {
public: 

    DCMotor(uint8_t pin){
        init(pin);
    }

    uint8_t init(uint8_t pin) {

        serv_port_pin_ = mapPinInnerRepresentation(pin);
        if (serv_port_pin_ == 255) {
            error_code_ = ErrorCodes::ERROR_INVALID_PIN;
            return error_code_;
        }
        motor_index = timer_control.getFreeMotorIndex(); //register motor

        if (motor_index < 0 ) {
            error_code_ = ErrorCodes::ERROR_NO_FREE_MOTOR;
            return error_code_;
        }; //error, no free motor
        timer_control.setMotorPortPin(motor_index, serv_port_pin_); //port B, pin 0
       
        return ErrorCodes::NO_ERROR;
    }

    void setTarget(int16_t value){
        if (value < -1000 || value > 1000){
            error_code_ = ErrorCodes::ERROR_INVALID_VALUE;
            return;
        }
        timer_control.setMotorValue(motor_index, value);
    }

    void setImmediate(int16_t value){
        if (value < -1000 || value > 1000){
            error_code_ = ErrorCodes::ERROR_INVALID_VALUE;
            return;
        }
        timer_control.setMotorValue(motor_index, value);
    }

    uint8_t getErrorCode() const {
        return error_code_;
    }
};


class ServoMotor : public IMotor {
public:

    ServoMotor(uint8_t pin) {

        serv_port_pin_ = mapPinInnerRepresentation(pin);
        motor_index = timer_control.getFreeMotorIndex(); //register motor

        if (motor_index < 0 ) return; //error, no free motor
        timer_control.setMotorPortPin(motor_index, serv_port_pin_); //port B, pin 0
    }

    ~ServoMotor() {
        //releases motor index
        if (motor_index < 0 ) return; //error, no motor to free
        timer_control.freeIndex(motor_index);
    }
    // Set target: for DC/stepper -> speed (-1000..1000), for servo -> angle (radians) depending on implementation
    void setTarget(int16_t value){
        timer_control.setMotorValue(motor_index, value);
    }
    
    // Immediately set output (no ramp) — useful for calibration/emergency
    void setImmediate(int16_t value){
        timer_control.setMotorValue(motor_index, value);
    }

    // Configure ramping: type and time to reach target (milliseconds)
    void configureRamp(int16_t rampTime_ms){
        ramp_.time_ms = rampTime_ms;
    }


private:
    int16_t motor_index = -1;
    uint8_t port_pin_ =0;
    MotorState state_;
    Ramp ramp_;
    int16_t currentOutput_; // current output value, ramped
    int16_t targetOutput_;  // target output value
};