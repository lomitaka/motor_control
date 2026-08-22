
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
constexpr uint16_t STEP_PULSE_TICKS = 20;       // Step pulse width (1.25 μs)
constexpr uint32_t TICKS_PER_SECOND = 16000000; // 16 MHz

// Static member definitions
volatile uint8_t StepperPositioning::stepper_count_ = 0;
volatile uint8_t StepperPositioning::step_pins_[MAX_STEPPERS] = {0, 0, 0, 0, 0};
volatile uint8_t StepperPositioning::dir_pins_[MAX_STEPPERS] = {0, 0, 0, 0, 0};
volatile int16_t StepperPositioning::current_speeds_[MAX_STEPPERS] = {0, 0, 0, 0, 0};
volatile int16_t StepperPositioning::target_speeds_[MAX_STEPPERS] = {200, 200, 200, 200, 200};

#define STOP 255
#define SLOW 10

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


void addToInterval2(volatile Interval & target, volatile Interval & new_current){
    
    if (target > new_current){
        //need to add to new_current
        uint32_t new_curr = ((uint32_t)new_current.skip_count << 16) + new_current.remainder;
        //add 8 percent of new_current to new_curr
        new_curr = new_curr + (new_curr >> 3);
        //handle top (converts it into stop)
        if (new_curr > (SLOW << 16)) {new_curr = (STOP << 16);}
        new_current.remainder = new_curr & 0xFFFF;
        new_current.skip_count = new_curr >> 16;
        
        if (new_current > target){
            new_current = target;
        }
        
    }else if (target < new_current){
        uint32_t new_curr = ((uint32_t)new_current.skip_count << 16) + new_current.remainder;
        //add 8 percent of new_current to new_curr
        
        new_curr = new_curr - (new_curr >> 3);
        //lower protectin
        if (new_curr < 200) {new_curr = 200;}
        new_current.remainder = new_curr & 0xFFFF;
        new_current.skip_count = new_curr >> 16;
        
        if (new_current < target){
            new_current = target;
        }
    }
}




void modifyIntervalWithOrder(volatile Interval & target, volatile Interval & new_current, int8_t diff){
    //diff - from old order to new order  = target_order- current_order
    if (diff > 0){
        new_current.skip_count = target.skip_count >> (diff);
        new_current.remainder = (target.skip_count >> (diff)) +  (target.remainder >> (diff));
    }else if (diff < 0){
        //need current make bigger than target
        new_current.skip_count = (target.skip_count << -diff) + (target.remainder >>  (16+diff));
        new_current.remainder = target.remainder << -diff;
    }
}


void addToInterval(volatile Interval & i, volatile Interval & val, bool subtract){
    if ((i.skip_count == STOP) && (subtract) ) { i.skip_count = SLOW; } 
    if ((i.skip_count != STOP) && ( i.skip_count >= SLOW) && (!subtract )) { i.skip_count = STOP; return; } 
    //add
    if (!subtract){
        uint16_t remainder_old = i.remainder;
        i.remainder = remainder_old + val.remainder;
        if (i.remainder < remainder_old && i.skip_count){
            i.skip_count++;
        }
        i.skip_count = i.skip_count + val.skip_count;
        //subtract
    }else {
        uint16_t remainder_old = i.remainder;
        i.remainder = remainder_old - val.remainder;
        if (i.remainder > remainder_old && i.skip_count == 0){
            i.remainder = 0;
            i.skip_count = 0;
        } else {
            i.skip_count--;
        }
        i.skip_count = min(i.skip_count,val.skip_count);
    }
}

// Internal ISR variables
namespace {

    volatile uint8_t update_rate[5] = {0, 0, 0, 0, 0};      // position of leading bit from current
    volatile uint8_t current_order[5] = {17, 17, 17, 17, 17};      // position of leading bit from current
    volatile uint8_t target_order[5] = {0, 0, 0, 0, 0};      // position of leading bit from target

    // Position tracking
    volatile int16_t remaining_ticks[5] = {0, 0, 0, 0, 0};      // Remaining steps to target (how many ticks to perform before stop)

    // Measuring ticks until edge high should occur (right now this interval is decreasing till zero)
    volatile Interval remaining[5] = { {0,0},{0,0},{0,0},{0,0},{0,0} };

    //length of the current interval - current length of the interval for next remaining update.
    volatile Interval current[5] = { {SLOW,0},{SLOW,0},{SLOW,0},{SLOW,0},{SLOW,0} };
    
    //length of the interval that is wished to achieve
    volatile Interval target[5] = { {0,0},{0,0},{0,0},{0,0},{0,0} };
    

    volatile int16_t braking_distance[5] = {0, 0, 0, 0, 0};    // Steps needed to stop
    
    // Timing (split to avoid 32-bit arithmetic)
    
    volatile uint16_t next_tick_count[5] = {0, 0, 0, 0, 0};     // Tick value within overflow period for next step
    //volatile uint32_t step_interval_ticks[5] = {16000, 16000, 16000, 16000, 16000}; // Full interval between steps (for recalculation)
    
    // Acceleration
    volatile uint16_t acceleration_rates[5] = {500, 500, 500, 500, 500}; // steps/s²
    volatile Interval interval_changes[5] = {{STOP,0},{STOP,0}, {STOP,0}, {STOP,0}, {STOP,0}};        // Pre-calculated speed change per accel cycle (avoids ISR division!)
    
    // Step pulse tracking
    volatile uint8_t step_pins_high_mask = 0;  // Bitmask: which motors need falling edge
    volatile bool event_presnet = false;  // if in current windows is planned any event
}

// Default constructor
StepperPositioning::StepperPositioning() 
    : step_pin_(0), dir_pin_(0),
      current_speed_sps_(0), target_speed_sps_(0), acceleration_(500),
      motor_index_(255) {
}

// Constructor with initialization
StepperPositioning::StepperPositioning(uint8_t step_pin, uint8_t dir_pin)
    : step_pin_(step_pin), dir_pin_(dir_pin),
      current_speed_sps_(0), target_speed_sps_(0), acceleration_(500),
      motor_index_(255) {
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
    
    // Configure pins
    digitalWrite(step_pin_, LOW);
    digitalWrite(dir_pin_, LOW);
    
    // Register motor in static arrays
    cli();
    step_pins_[motor_index_] = step_pin_;
    dir_pins_[motor_index_] = dir_pin_;
    current_speeds_[motor_index_] = 0;
    target_speeds_[motor_index_] = 0;
    remaining_ticks[motor_index_] = 0;
    braking_distance[motor_index_] = 0;
    //overflow_skip_count[motor_index_] = 0;        // No overflow skip initially
    next_tick_count[motor_index_] = TCNT1 + 10000; // Start in future
    //step_interval_ticks[motor_index_] = 16000;    // 1 kHz initial
    sei();
    
    // Initialize timer if not already done
    if (!SteppTimerControl::isInitialized()) {
        SteppTimerControl::Timer1_Init();
    }
    
    // Pre-calculate acceleration rate
    updateAccelerationRate();
    
    return 0;
}

uint8_t StepperPositioning::setTargetTicks(int16_t steps) {   
    cli();
    //int16_t current_remaining = remaining_ticks[motor_index_];
    int16_t brake_dist = braking_distance[motor_index_];
    sei();
    
    // Check if we have enough distance to decelerate
    int16_t abs_steps = (steps < 0) ? -steps : steps;
    if (abs_steps < brake_dist) {
        return 1; // ERROR: Cannot execute with current deceleration
    }
    
    // Set new target
    cli();
    remaining_ticks[motor_index_] = steps;
    sei();
    
    // Update direction
    updateDirection();
    updateStepInterval();
    updateBrakingDistance();
    
    return 0;
}

uint8_t StepperPositioning::setSpeed(int16_t steps_per_sec) {
    // Clamp to valid range: -5000 to +5000 steps/s
    if (steps_per_sec > 5000) steps_per_sec = 5000;
    if (steps_per_sec < -5000) steps_per_sec = -5000;
    cli();
    target_speeds_[motor_index_] = steps_per_sec;
    sei();

    
    // Update direction
    updateDirection();
    
    // Recalculate braking distance and step interval
    updateStepInterval();
    updateBrakingDistance();
    
    return 0;
}

uint8_t StepperPositioning::addTargetTicks(int16_t steps) {
    
    //int16_t current_remaining = remaining_ticks[motor_index_];
    int16_t brake_dist = braking_distance[motor_index_];

    // Check if we have enough distance to decelerate
    int16_t abs_steps = (steps < 0) ? -steps : steps;
    if (abs_steps < brake_dist) {
        return 1; // ERROR: Cannot execute with current deceleration
    }
    
    // Set new target
    cli();
    int16_t new_target = remaining_ticks[motor_index_] + steps;
    remaining_ticks[motor_index_] = new_target;
    sei();
    
    // Update direction
    updateDirection();
    updateStepInterval();
    updateBrakingDistance();
    return 0;

}

void StepperPositioning::setAcceleration(uint16_t percent, uint8_t rate) {
    acceleration_ = percent;
    cli();
    acceleration_rates[motor_index_] = 0;//steps_per_sec2;
    sei();
    updateAccelerationRate();
    updateBrakingDistance();
}

void StepperPositioning::setImmediateTicks(int16_t steps) {
    // Clamp to valid range
    if (steps > 5000) steps = 5000;
    if (steps < -5000) steps = -5000;
    
    cli();
    remaining_ticks[motor_index_] = steps;
    
    // Set speed immediately (no acceleration)
    if (steps != 0) {
        // Calculate speed to complete steps
        int16_t direction = (steps > 0) ? 1 : -1;
        int16_t abs_steps = (steps < 0) ? -steps : steps;
        
        // Use target speed, or default to reasonable speed
        int16_t speed = target_speeds_[motor_index_];
        if (speed == 0) speed = 1000; // Default 1000 steps/s
        
        current_speeds_[motor_index_] = direction * ((speed < 0) ? -speed : speed);
    } else {
        current_speeds_[motor_index_] = 0;
    }
    sei();
    
    updateDirection();
    updateStepInterval();
    updateBrakingDistance();
}

bool StepperPositioning::isMoving() {
    cli();
    bool moving = (remaining_ticks[motor_index_] != 0) || (current_speeds_[motor_index_] != 0);
    sei();
    return moving;
}

void StepperPositioning::updateStepInterval() {
    int16_t speed = target_speeds_[motor_index_];
    
    if (speed == 0) {
        cli();
        //sets skip count for 255 > very high, (interpreted as stop)
        target[motor_index_].remainder = 0;
        target[motor_index_].skip_count = STOP;
        sei();
        return;
    }
    
    // Calculate interval in ticks
    // interval_ticks = TICKS_PER_SECOND / steps_per_sec
    uint16_t abs_speed = (speed < 0) ? -speed : speed;
    if (abs_speed == 0) abs_speed = 1;
    
    uint32_t interval = TICKS_PER_SECOND / abs_speed;
    if (interval < 100) interval = 100; // Safety: min 100 ticks (6.25μs)
    // Note: interval can be > 65535 (e.g., for very slow speeds)
    
    uint32_t int_copy = interval;
    uint8_t order = 0;
    while(int_copy != 0){
        int_copy = int_copy >> 1;
        order++;
    }
    
    cli();
    //step_interval_ticks[motor_index_] = interval;
    target_order[motor_index_] = order;
    target[motor_index_].skip_count = interval / 65535;
    target[motor_index_].remainder = interval % 65535;

    //remaining[motor_index_].remainder = mabs(target[motor_index_].remainder -  current[motor_index_].remainder)/250;
    //current[motor_index_].remainder = mabs(target[motor_index_].remainder -  current[motor_index_].remainder)/250;
    sei();
}


void StepperPositioning::updateDirection() {
    int16_t speed = target_speeds_[motor_index_];
    
    if (speed >= 0) {
        digitalWrite(dir_pin_, HIGH);  // Clockwise
    } else {
        digitalWrite(dir_pin_, LOW);   // Counter-clockwise
    }
}

void StepperPositioning::updateAccelerationRate() {
    // Pre-calculate speed change per acceleration cycle (4.096ms)
    // This avoids expensive multiplication in ISR!
    // speed_change = acceleration × 0.004096s
    //              = acceleration × 4096 / 1000000
    
    uint32_t accel = acceleration_;
    //uint16_t speed_change = (accel * 4096) / 1000000;
    int16_t cs =current_speeds_[motor_index_]; // steps per second
    if (cs == 0) {cs = 1;}
    int16_t sc = mabs(target_speeds_[motor_index_] - cs); //speed change
    int32_t interval_change = (TICKS_PER_SECOND * accel) / (cs*cs+cs*accel);
    
    //if (speed_change == 0) speed_change = 1; // Minimum change of 1 step/s
    Interval i;
    i.skip_count = interval_change / 65536;
    i.remainder = interval_change % 65536;
    cli();
    interval_changes[motor_index_] = i;
    sei();

}

void StepperPositioning::updateBrakingDistance() {
    // Calculate braking distance: steps needed to decelerate to zero
    // Using kinematic equation: v² = v₀² + 2as
    // Solving for s: s = v² / (2a)
    
    cli();
    int16_t speed = current_speeds_[motor_index_];
    uint16_t accel = acceleration_rates[motor_index_];
    sei();
    
    if (accel == 0) accel = 1;
    
    uint16_t abs_speed = (speed < 0) ? -speed : speed;
    
    // braking_distance = speed² / (2 × acceleration)
    uint32_t speed_squared = (uint32_t)abs_speed * abs_speed;
    uint16_t brake_dist = speed_squared / (2 * accel);
    
    cli();
    braking_distance[motor_index_] = brake_dist;
    sei();
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

    uint16_t next_event = 65535; // Max value
    event_presnet = false;

    for (uint8_t i = 0; i < StepperPositioning::stepper_count_; i++) {
        // Decrement overflow skip counter
        if (remaining[i].skip_count != STOP && remaining[i].skip_count != 0) {
            remaining[i].skip_count--;
        }
        
        // Handle acceleration/deceleration
        //int16_t current = StepperPositioning::current_speeds_[i];
        //int16_t target = StepperPositioning::target_speeds_[i];
        int16_t remaining_t = remaining_ticks[i];
        int16_t brake_dist = braking_distance[i];
        
        // Check if we need to start braking
        int16_t abs_remaining = (remaining_t < 0) ? -remaining_t : remaining_t;
        if (abs_remaining <= brake_dist) {
            // Start deceleration (pick next target that is stop)
            target[i].remainder = 0;
            target[i].skip_count = STOP;
        }
        if (current[i] != target[i]){
            addToInterval2(target[i],current[i]);
        }
        /*if (current_order[i] != target_order[i]){
            if (current_order[i] > target_order[i]){
                current_order[i]--;
                
                modifyIntervalWithOrder(target[i],current[i],target_order[i] - current_order[i]);
                modifyIntervalWithOrder(target[i],current[i],target_order[i] - current_order[i]);
            }else {
            
                current_order[i]++;
            }
            
        }*/
        
        // Accelerate/decelerate
        //if (current != target) {
        if (current_order[i] == target_order[i] && current[i] != target[i]){
            //volatile Interval & change = interval_changes[i];
            
            current[i] = target[i];        
        }

        //if remaining does not catchesup with udpate (current).. set remainin to current.
        //if (remaining[i] > current[i]){ remaining[i] = current[i];}

        //is timer enabled?
        if (current[i].skip_count != STOP ) {
            //Take nearest event
            if (remaining[i].skip_count == 0 && remaining[i].remainder < next_event) {
                next_event = remaining[i].remainder;
                event_presnet = true;
            }
        }
    }


    uint16_t current_time = TCNT1;
            // Schedule next OCRA
    if (event_presnet) {//TODO FIX
        uint16_t safe_next = (next_event > current_time) ? (next_event - current_time) : TIMER_GUARD_TICKS;
        if (safe_next < TIMER_GUARD_TICKS) safe_next = TIMER_GUARD_TICKS;
        OCR1A = current_time + safe_next;
    } 
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
    if (!event_presnet) {return;}
    uint16_t current_time = TCNT1;
    uint16_t next_event = 65535; // Max value
    step_pins_high_mask = 0;
    
    // Process each active motor
    for (uint8_t i = 0; i < StepperPositioning::stepper_count_; i++) {
        // Skip if motor not configured
        if (StepperPositioning::step_pins_[i] == 0) continue;
        
        // Check if this motor needs a step now (overflow_skip_count == 0 and time reached)
        if (remaining[i].skip_count == 0 && remaining_ticks[i] != 0) {
            if (remaining[i].remainder <= current_time + TIMER_GUARD_TICKS) {
                // Generate step
                digitalWrite(StepperPositioning::step_pins_[i], HIGH);
                step_pins_high_mask |= (1 << i);
                
                // Decrement remaining ticks
                if (remaining_ticks[i] > 0) {
                    remaining_ticks[i]--;
                } else if (remaining_ticks[i] < 0) {
                    remaining_ticks[i]++;
                }
                
                remaining[i].skip_count = current[i].skip_count;
                uint16_t remainder_new =remaining[i].remainder+ current[i].remainder;
                //in case of overflow add additional number to... a vis ty co. ja to prevedu na uinty.
                if (remainder_new < remaining[i].remainder ){
                    remaining[i].skip_count++;
                }
                remaining[i].remainder =  remainder_new;
                // Calculate next step time: split interval into overflow count and tick count
                //uint32_t interval = step_interval_ticks[i];
                //overflow_skip_count[i] = interval / 65536;  // Integer division
                //next_tick_count[i] = interval % 65536;       // Remainder
                
                // If next_tick_count is very small, it might trigger immediately - add to current time
                //if (overflow_skip_count[i] == 0) {
                //    next_tick_count[i] = current_time + (uint16_t)interval;
                //}
            }
        }

        // Stop if reached target
        if (remaining_ticks[i] == 0) {
            StepperPositioning::current_speeds_[i] = 0;
        }

        // Find nearest next event (only motors with overflow_skip_count == 0)
        if (remaining_ticks[i] != 0 && remaining[i].skip_count == 0) {
            if (remaining[i].remainder > current_time && remaining[i].remainder < next_event) {
                next_event = remaining[i].remainder;
            }
        }
    }
    
    // Schedule next OCRA
    if (next_event < 65535) {//TODO FIX
        uint16_t safe_next = (next_event > current_time) ? (next_event - current_time) : TIMER_GUARD_TICKS;
        if (safe_next < TIMER_GUARD_TICKS) safe_next = TIMER_GUARD_TICKS;
        OCR1A = current_time + safe_next;
    } 
    
    // Schedule OCRB for falling edges
    if (step_pins_high_mask != 0) {
        OCR1B = current_time + STEP_PULSE_TICKS;
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


DebugInfo di;
di.remainining_interval = remaining[0].skip_count * 65536 + remaining[0].remainder;
if (remaining[0].skip_count == STOP){
    di.remainining_interval = -1;
}

di.current_interval = current[0].skip_count * 65536 + current[0].remainder;
if (current[0].skip_count == STOP){
    di.current_interval = -1;
}

di.target_interval = target[0].skip_count * 65536 + target[0].remainder;
if (target[0].skip_count == STOP){
    di.target_interval = -1;
}

di.tar_rate = target_order[0];
di.curr_rate = current_order[0];

return di;
//volatile int16_t StepperPositioning::current_speeds_[MAX_STEPPERS] = {0, 0, 0, 0, 0};
//volatile int16_t StepperPositioning::target_speeds_[MAX_STEPPERS] = {200, 200, 200, 200, 200};

}
