
#ifdef SIMULATION_MODE
    #include "../simulator/avr_mock.h"
#else
    #include <avr/io.h>
    #include <avr/interrupt.h>
#endif

#include "motor_control/stepper_positioning.h"
#include "internals/stepp_timer_control.h"
#include "internals/arduino.h"
#include "internals/fces.h"

using namespace motor_control_internals;

/**
 * @file stepp_control.cpp
 * @brief Stepper motor positioning control implementation
 * 
 * Timer Configuration:
 * - No prescaler → Timer freq = 16 MHz, tick period = 62.5 ns
 * - OCR1A: Dynamically scheduled for next step event (like DC control)
 * - OCR1B: Scheduled few ticks after OCR1A for falling edges
 * 
 * Architecture:
 * - Up to 5 stepper motors with independent positioning
 * - Each motor tracks: position, speed, acceleration, braking distance
 * - Trapezoidal motion profile: accel → constant → decel
 * - ISR schedules next event based on nearest motor's next step
 * 
 * Supports precise positioning with configurable acceleration.
 */

// Timer constants
constexpr uint16_t TIMER_GUARD_TICKS = 100;     // Safety margin (6.25 μs)
constexpr uint16_t STEP_PULSE_TICKS = 100;       // Step pulse width (1.25 μs)
constexpr uint32_t TICKS_PER_SECOND = 16000000; // 16 MHz

// Static member definitions
volatile uint8_t StepperPositioning::stepper_count_ = 0;
volatile uint8_t StepperPositioning::step_pins_[MAX_STEPPERS] = {0, 0, 0, 0, 0};
volatile uint8_t StepperPositioning::dir_pins_[MAX_STEPPERS] = {0, 0, 0, 0, 0};
volatile uint8_t StepperPositioning::dir_pins_value_[MAX_STEPPERS] = {0, 0, 0, 0, 0};
volatile uint16_t StepperPositioning::target_speeds_stp_ps_[MAX_STEPPERS] = {200, 200, 200, 200, 200};

#define STOP 250
#define SLOW 9
#define VERY_SLOW 244
#define VERY_SLOW2 (uint32_t)244 << 16

struct Interval {
    uint8_t skip_count;//255 for full stop
    uint16_t remainder;

    // Elegantnější cesta: Přetížení operátoru <
    // Pozor: Operátor v C++ standardně bere jen pravou stranu, 
    // takže divider musíme předat jinak, nebo předpokládat fixní divider.
    // Zde ukázka s předpokladem stejného divideru pro obě struktury:
    bool operator<(const volatile Interval& other) const volatile {
        uint8_t sc1 = skip_count;
        uint8_t sc2 = other.skip_count;
        if (sc1 != sc2) return sc1 < sc2;
        
        uint16_t rem1 = remainder;
        uint16_t rem2 = other.remainder;
        return rem1 < rem2;
    }

    // Přetížení operátoru rovnosti (==)
    bool operator==(const volatile Interval& other) const volatile {
        return (skip_count == other.skip_count) && (remainder == other.remainder);
    }

    // Přetížení operátoru nerovnosti (!=)
    bool operator!=(const volatile Interval& other) const volatile {
        return (skip_count !=  other.skip_count) || (remainder != other.remainder);
    }
    
    bool operator>(const volatile Interval& other) const volatile {
    uint8_t sc1 = skip_count;
    uint8_t sc2 = other.skip_count;
    if (sc1 != sc2) return sc1 > sc2; // Větší než (>)
    
    uint16_t rem1 = remainder;
    uint16_t rem2 = other.remainder;
    return rem1 > rem2; // Větší než (>)
    }

    volatile Interval& operator=(const volatile Interval& other) volatile {
        // Bezpečně načteme hodnoty z 'other' do lokálních proměnných 
        // (pokud by 'other' byl náhodou také volatile)
        uint8_t sc = other.skip_count;
        uint16_t rem = other.remainder;

        // Zapíšeme je do našich volatile položek
        skip_count = sc;
        remainder = rem;

        // Operátor přiřazení v C++ standardně vrací referenci na sebe, 
        // abyste mohli řetězit přiřazení typu a = b = c;
        return *this;
    }

};

//-1 interval extend, 1 interval schrinks
int8_t addToInterval(volatile Interval & target, volatile Interval & new_current, uint8_t accel_shif){
    //modifier -number from 1-10 for more aggresive interval update
    if (target > new_current){
        //need to add to new_current
        uint32_t new_curr = ((uint32_t)new_current.skip_count << 16) + new_current.remainder;
        //add 8 percent of new_current to new_curr
        new_curr = new_curr + (new_curr >> accel_shif);
        //handle top (converts it into stop)target_speeds_stp_ps_
        if (new_current.skip_count > VERY_SLOW ) {new_curr = ((uint32_t)VERY_SLOW << 16);}
        new_current.remainder = new_curr & 0xFFFF;
        new_current.skip_count = new_curr >> 16;
        
        if (new_current > target){
            new_current.remainder = target.remainder;
            new_current.skip_count = target.skip_count;
        }
        return -1;
    }else if (target < new_current){
        uint32_t new_curr = ((uint32_t)new_current.skip_count << 16) + new_current.remainder;
        //add 8 percent of new_current to new_curr
        
        new_curr = new_curr - (new_curr >> accel_shif);
        //lower protectin
        if (new_curr < 200) {new_curr = 200;}
        new_current.remainder = new_curr & 0xFFFF;
        new_current.skip_count = new_curr >> 16;
        
        if (new_current < target){
            new_current.remainder = target.remainder;
            new_current.skip_count = target.skip_count;
        }
        return 1;
    }
    return 0;
}



// Internal ISR variables
namespace {

    volatile Interval spinup_interval = {9,0};
    // Position tracking
    volatile int16_t remaining_ticks[5] = {0, 0, 0, 0, 0};      // Remaining steps to target (how many ticks to perform before stop)

    // Measuring ticks until edge high should occur (right now this interval is decreasing till zero)
    volatile Interval remaining[5] = { {0,0},{0,0},{0,0},{0,0},{0,0} };
    volatile uint8_t next_ = 0; //mask of  indexes to be processed in current window. (always set on overflow, may be updated on OCRA), 0 for no update
    //length of the current interval - current length of the interval for next remaining update.
    volatile Interval current[5] = { {SLOW,0},{SLOW,0},{SLOW,0},{SLOW,0},{SLOW,0} };
    
    //length of the interval that is wished to achieve
    volatile Interval target[5] = { {0,0},{0,0},{0,0},{0,0},{0,0} };
    
    volatile uint16_t acceleration_shift[5] = {3,3,3,3,3};      ///2-6 adds 50% to 3% percent to acceleration speed (default 3 ~ 12%)
    volatile int16_t braking_distance[5] = {0, 0, 0, 0, 0};    // Steps needed to stop
    volatile int8_t accel_type[5] = {0, 0, 0, 0, 0};    // 1 ramp up, 0  const speed, -1 ramp down
    volatile uint8_t enabled_mask = false;    // bitmask for enabled motors [lower index - motor 0]

    
    // Step pulse tracking
    volatile uint8_t step_pins_high_mask = 0;  // Bitmask: which motors need falling edge
    volatile bool event_present = false;  // if in current windows is planned any event
}

#define MOTOR_ENABLED(num)  enabled_mask & (1 << num)
#define BIT_SET(num,bit)  num & (1 << bit)

// Default constructor
StepperPositioning::StepperPositioning() 
    : step_pin_(0), dir_pin_(0),
      current_speed_sps_(0), target_speed_sps_(0), motor_index_(255) {
}

// Constructor with initialization
StepperPositioning::StepperPositioning(uint8_t step_pin, uint8_t dir_pin)
    : step_pin_(step_pin), dir_pin_(dir_pin),
      current_speed_sps_(0), target_speed_sps_(0), motor_index_(255) {
    init(step_pin, dir_pin);
}

uint8_t StepperPositioning::init(uint8_t step_pin, uint8_t dir_pin) {
    step_pin_ = step_pin;
    dir_pin_ = dir_pin;
    
    // Find free motor slot
    if (stepper_count_ >= MAX_STEPPERS) {
        return 1; // ERROR_NO_FREE_MOTOR
    }
    
    motor_index_ = stepper_count_;
    stepper_count_++;
    
    // Configure pins owned by this driver before writing their initial state.
    pinMode(step_pin_, OUTPUT);
    pinMode(dir_pin_, OUTPUT);
    digitalWrite(step_pin_, LOW);
    digitalWrite(dir_pin_, LOW);
    
    // Register motor in static arrays
    uint8_t saved_sreg = SREG;
    cli();
    step_pins_[motor_index_] = step_pin_;
    dir_pins_[motor_index_] = dir_pin_;
    target_speeds_stp_ps_[motor_index_] = 200;
    remaining_ticks[motor_index_] = 0;
    braking_distance[motor_index_] = 0;
    SREG = saved_sreg;
    
    // Initialize timer if not already done
    if (!SteppTimerControl::isInitialized()) {
        SteppTimerControl::Timer1_Init();
    }
    
    // Pre-calculate acceleration rate
    updateAccelerationRate();
    
    return 0;
}
void StepperPositioning::setSpinupSpeedTicks(uint16_t steps_per_sec){   
    if (steps_per_sec == 0) { steps_per_sec = 1; }
    uint32_t interval = TICKS_PER_SECOND / steps_per_sec;
    if (interval < 100) interval = 100; // Safety: min 100 ticks (6.25μs)
    // Note: interval can be > 65535 (e.g., for very slow speeds)
    cli();   
    //step_interval_ticks[motor_index_] = interval;
    spinup_interval.skip_count = interval / 65536;
    spinup_interval.remainder = interval % 65536;
    sei();

}
void StepperPositioning::setTargetTicks(int16_t steps) {   

        
    // Set new target
    cli();
    remaining_ticks[motor_index_] = steps;
    sei();
    
    // Update direction
    //updateDirection();
    updateStepInterval();

}

void StepperPositioning::setSpeed(uint16_t steps_per_sec) {
    // Clamp to valid range: 0 to +5000 steps/s
    if (steps_per_sec > 5000) steps_per_sec = 5000;
    cli();
    target_speeds_stp_ps_[motor_index_] = steps_per_sec;
    sei();

        
    // Recalculate braking distance and step interval
    updateStepInterval();
    
}

void StepperPositioning::addTargetTicks(int16_t steps) {
    
    // Set new target
    cli();
    int16_t new_target = remaining_ticks[motor_index_] + steps;
    remaining_ticks[motor_index_] = new_target;
    sei();
    
    // Update direction
    //updateDirection();
    updateStepInterval();
}

void StepperPositioning::setAcceleration(uint8_t percent_increase) {
    if (percent_increase < 1) {percent_increase = 1;}
    if (percent_increase > 5) {percent_increase = 5;}
    
    cli();
    acceleration_shift[motor_index_] =7- percent_increase;
    sei();
    //updateAccelerationRate();
}

void StepperPositioning::setImmediateTicks(int16_t steps) {
    // Clamp to valid range
    if (steps > 5000) steps = 5000;
    if (steps < -5000) steps = -5000;
    
    cli();
    remaining_ticks[motor_index_] = steps;
    
    sei();
    
    updateDirection();
    updateStepInterval();
}

bool StepperPositioning::isMoving() {
    cli();
    bool moving = (remaining_ticks[motor_index_] != 0);
    sei();
    return moving;
}

void StepperPositioning::updateStepInterval() {
    uint16_t speed = target_speeds_stp_ps_[motor_index_];
    
    if (speed == 0) {
        cli();
        //sets skip count for 255 > very high, (interpreted as stop)
        target[motor_index_].remainder = 0;
        target[motor_index_].skip_count = VERY_SLOW;
        sei();
        return;
    }
        
    uint32_t interval = TICKS_PER_SECOND / speed;
    if (interval < 100) interval = 100; // Safety: min 100 ticks (6.25μs)
    // Note: interval can be > 65535 (e.g., for very slow speeds)
        
    cli();
    //step_interval_ticks[motor_index_] = interval;
    target[motor_index_].skip_count = interval / 65536;
    target[motor_index_].remainder = interval % 65536;
    if (current[motor_index_] > spinup_interval){
        current[motor_index_].skip_count = STOP; 
    }
    
    
    enabled_mask |= 1 << motor_index_;

    sei();
}


void StepperPositioning::updateDirection() {
    int16_t speed = target_speeds_stp_ps_[motor_index_];
    
    if (speed >= 0) {
        digitalWrite(dir_pin_, HIGH);  // Clockwise
        dir_pins_value_[motor_index_] = 0;
    } else {
        digitalWrite(dir_pin_, LOW);   // Counter-clockwise
        dir_pins_value_[motor_index_] = 1;
    }
}

void StepperPositioning::updateAccelerationRate() {
    // Pre-calculate speed change per acceleration cycle (4.096ms)
    // This avoids expensive multiplication in ISR!
    // speed_change = acceleration × 0.004096s
    //              = acceleration × 4096 / 1000000
       


}


void setOCRA(uint8_t stepper_count){
    uint16_t next_event = 65535; // Max value
    uint8_t next_index = 0;
    event_present = false;

    for (uint8_t i = 0; i < stepper_count; i++) 
    {

        //iterating over evetns. want to take event that occures first. 
        //searching fo minimum event. 
        //if event with no 20 found and another with 20 -> both are set to process
        //if 20, 20, 19  -> 19 disables 2x 20
        // 
        //is timer enabled?
        if (MOTOR_ENABLED(i) && remaining[i].skip_count == 0 && remaining_ticks[i] != 0) {
            //Hanlde events, that occurs in the same tick
            if (remaining[i].remainder == next_event && event_present) {
                next_index |= 1 << i;
            }
            //there is better event. 
            if (remaining[i].remainder < next_event) {
                next_event = remaining[i].remainder;
                //clears next event !
                next_index = 1 << i;
                event_present = true;
            }
        }
    }

    // Schedule next OCRA for earliest event. 
    uint16_t current_time = TCNT1;
    
    if (event_present) {
        uint16_t safe_next = (next_event > current_time) ? (next_event - current_time) : TIMER_GUARD_TICKS;
        if (safe_next < TIMER_GUARD_TICKS) safe_next = TIMER_GUARD_TICKS;
        OCR1A = current_time + safe_next;
        next_ = next_index; 
    } 


}

/**
 * @brief Timer1 Overflow ISR - Acceleration/deceleration and overflow counting
 * 
 * Called every 65536 ticks (4.096 ms). This ISR:
 * 1. Handles acceleration/deceleration for all motors
 * 2. Decrements overflow_skip_count for pending steps
 * 3. Recalculates step intervals when speed changes
 */
void OnTimer1StepperPositioningOverflow(){  

    for (uint8_t i = 0; i < StepperPositioning::stepper_count_; i++) {
        // Decrement overflow skip counter
        if (remaining[i].skip_count != STOP && remaining[i].skip_count != 0) {
            remaining[i].skip_count--;
        }
        
        int16_t remaining_t = remaining_ticks[i];
        //compensate for errors
        if (braking_distance[i] < 0) {braking_distance[i] = 0;}
        int16_t brake_dist = braking_distance[i];
        
        
        // Check if we need to start braking
        int16_t abs_remaining = (remaining_t < 0) ? -remaining_t : remaining_t;
        //TODO ENABLE BRAKING DISTANCE
        if (abs_remaining <= brake_dist) {
            // Start deceleration (pick next target that is stop)
            target[i].remainder = 0;
            target[i].skip_count = SLOW;
        }

        

        uint8_t current_direction_pos = StepperPositioning::dir_pins_value_[i];
        //if (i == 0 && current_direction_pos){std::cout << "current pose: 1" << std::endl;}
        //if (i == 0 && !current_direction_pos){std::cout << "current pose: 0" << std::endl;}
        
        //if there is direction mismatch. i should slow down.  and if i am already slowed down, change direction.
        if (((remaining_ticks[i] > 0) && current_direction_pos) ||
            ((remaining_ticks[i] < 0) && !current_direction_pos)){
            //current is longer (slower) than spinup interval
            // slow down.
            accel_type[i] = addToInterval(spinup_interval,current[i],acceleration_shift[i]);
            
            if (!(current[i] < spinup_interval )){
                //change direction of direction pin
                digitalWrite(StepperPositioning::dir_pins_[i],1-current_direction_pos);
                StepperPositioning::dir_pins_value_[i] = 1-current_direction_pos;
            }
        }else 
        
           /* std::cout << "target: " << (((uint32_t)target[i].skip_count << 16) + target[i].remainder)
                      << " current: " << (((uint32_t)current[i].skip_count << 16) + current[i].remainder)
                      << std::endl;*/
            accel_type[i] = addToInterval(target[i],current[i],acceleration_shift[i]);
            /*std::cout << "target: " << (((uint32_t)target[i].skip_count << 16) + target[i].remainder)
                      << " current: " << (((uint32_t)current[i].skip_count << 16) + current[i].remainder)
                      << std::endl;*/
            
            // if (current->skip_count > SLOW  && remaining_ticks[i] == 0) {new_curr = (STOP << 16);}

        
    }

    setOCRA(StepperPositioning::stepper_count_);
}


/**
 * @brief Timer1 OCRA ISR - Rising edge generation and next event scheduling
 * 
 * Called when TCNT1 reaches OCR1A. This ISR:
 * 1. Generates STEP rising edges for motors whose time has arrived (overflow_skip_count == 0 && TCNT1 >= next_tick_count)
 * 2. Updates motor position (remaining_ticks)
 * 3. Schedules next OCR1A for the nearest motor event
 * 4. Schedules OCR1B for falling edges (few ticks later)
 * 
 * Uses 16-bit arithmetic only for speed on 8-bit AVR.
 */
void OnTimer1StepperPositioningOCRA() {
    if (!event_present) {return;}
    step_pins_high_mask = 0;
    event_present = false;

    //probably redundant with event_preset
    if (next_ == 0){ return;}

    // Check if this motor needs a step now (overflow_skip_count == 0 and time reached)
    for (int i = 0; i < 5; i++)
    {
        if (!(BIT_SET(next_,i))){continue;}
        //there are still ticks to perform before stop
        if (remaining_ticks[i] != 0) {
            
            // Generate step
            digitalWrite(StepperPositioning::step_pins_[i], HIGH);
            step_pins_high_mask |= (1 << i);
            
            // Decrement remaining ticks
            if (remaining_ticks[i] > 0) {
                remaining_ticks[i]--;
            } else if (remaining_ticks[i] < 0) {
                remaining_ticks[i]++;
            }

            //disable motor if ticks are zero
            if (remaining_ticks == 0){
                //clear breaking distance
                braking_distance[i] = 0;
                enabled_mask &= ~(1 << i);
            }

            //each speedup increases number, and speed down decreases. intent is to use for ramp down tick counting.
            braking_distance[i] = braking_distance[i] + accel_type[i];

            uint32_t remainder_new = ((uint32_t)remaining[i].skip_count << 16) + remaining[i].remainder;
            remainder_new = remainder_new + ((uint32_t)current[i].skip_count << 16) + current[i].remainder;
            //if it too slow, then stop it. 
            if (remainder_new > VERY_SLOW2) {
                remaining[i].remainder = 0;
                remaining[i].skip_count = STOP;
            }else {
                remaining[i].remainder = remainder_new & 0xFFFF;
                remaining[i].skip_count = (remainder_new >> 16);
            }
            
        }
    }

    setOCRA(StepperPositioning::stepper_count_);
    
    // Schedule OCRB for falling edges
    if (step_pins_high_mask != 0) {
        OCR1B = TCNT1 + STEP_PULSE_TICKS;
    }
}

/**
 * @brief Timer1 OCRB ISR - Falling edge generation
 * 
 * Called few ticks after OCRA. Clears all STEP pins that were set HIGH.
 */
void OnTimer1StepperPositioningOCRB() {
    // Clear all pins that were set HIGH
    for (uint8_t i = 0; i < StepperPositioning::stepper_count_; i++) {
        if (step_pins_high_mask & (1 << i)) {
            digitalWrite(StepperPositioning::step_pins_[i], LOW);
        }
    }
    step_pins_high_mask = 0;
}


DebugInfo GetDebugInfo(){

  /*  volatile Interval remaining[5] = { {0,0},{0,0},{0,0},{0,0},{0,0} };

    //length of the current interval - current length of the interval for next remaining update.
    volatile Interval current[5] = { {255,0},{255,0},{255,0},{255,0},{255,0} };
    
    //length of the interval that is wished to achieve
    volatile Interval target[5] = { {0,0},{0,0},{0,0},{0,0},{0,0} };
    

    volatile int16_t braking_distance[5] = {0, 0, 0, 0, 0};    // Steps needed to stop
    
    // Timing (split to avoid 32-bit arithmetic)
    
    volatile uint16_t next_tick_count[5] = {0, 0, 0, 0, 0};     // T*/

cli();
DebugInfo di;
di.remainining_interval = ((int64_t)remaining[0].skip_count << 16) + remaining[0].remainder;
if (remaining[0].skip_count == STOP){
    di.remainining_interval = -1;
}

di.current_interval = ((int64_t)current[0].skip_count << 16) + current[0].remainder;
if (current[0].skip_count == STOP){
    di.current_interval = -1;
}


di.target_interval = ((int64_t)target[0].skip_count <<16) + target[0].remainder;
if (target[0].skip_count == STOP){
    di.target_interval = -1;
}

di.braking_distance = braking_distance[0];
sei();
return di;
//volatile int16_t StepperPositioning::current_speeds_[MAX_STEPPERS] = {0, 0, 0, 0, 0};
//volatile int16_t StepperPositioning::target_speeds_[MAX_STEPPERS] = {200, 200, 200, 200, 200};

}
