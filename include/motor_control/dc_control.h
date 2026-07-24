#ifndef DC_CONTROL_H
#define DC_CONTROL_H

#include <stdint.h>
#include "motor.h"

class DCControl {
public:
    DCControl();

    DCControl(uint8_t pin_pwm, uint8_t pin_direction);

    uint8_t init(uint8_t pin_pwm, uint8_t pin_direction);

    void setTarget(int16_t value);

    void setImmediate(int16_t value);

    //in case of failure, get last error code
    uint8_t getLastError();

  
private:
    //dc motors:
    
    // Set motor value in range [-1000, 1000] for given index (0-4)
    static void setDCMotorValue(uint8_t index, int16_t value);
    
    // Overloaded version: set port and pin using single byte (port in high nibble, pin in low nibble)
    static void setDCMotorPortPin(uint8_t index, uint8_t port_pwm_pin, uint8_t port_dir_pin );

    // get index of first free motor
    static int8_t getDCFreeMotorIndex();
    //detaches motor from given index
    void freeDCIndex(uint8_t index);


    volatile static uint8_t dc_motor_count_;
    //motor values currently set (-1000 to 1000)
    volatile static uint16_t dc_motors_[5];
    //used to copy dc_motors_ and use these values, for next time cycle. 
    volatile static uint16_t dc_motors_buffer_[5];
    //port and pin for each motor (0=disabled, else port in high nibble, pin in low nibble)
    volatile static uint8_t dc_port_pwm_pin_[5];

    /// @brief pins where direction is set
    volatile static uint8_t dc_port_dir_pin_[5];

    friend void OnTimer1OwerflowDC();
    friend void OnTimer1CompareMatchDC();
    friend bool isSmaller(uint8_t index1, uint8_t index2);

    //MotorProps props;
    uint8_t port_pwm_index_;
    uint8_t port_dir_index;
    //which index from static arrays belongs to this instance
    uint8_t motor_index_;
    uint8_t error_code_;
};

#endif // DC_CONTROL_H
