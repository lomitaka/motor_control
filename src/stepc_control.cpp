
#ifdef SIMULATION_MODE
    #include "../simulator/avr_mock.h"
#else
    #include <avr/io.h>
    #include <avr/interrupt.h>
#endif

#include "motor_control/stepper_continuous.h"
#include "internals/stepc_timer_control.h"
#include "internals/arduino.h"
#include "internals/fces.h"

using namespace motor_control_internals;

/**
 * @file step_control.cpp
 * @brief Stepper motor control implementation using accumulator approach
 * 
 * Timer Configuration:
 * - Prescaler 256 → Timer freq = 62.5 kHz, tick period = 16 μs
 * - ISR frequency: 8928 Hz (every 112μs, 7 ticks)
 * - Accumulator approach: Precise timing for multiple steppers
 * 
 * Performance Optimization:
 * - Uses 16-bit tick arithmetic instead of 32-bit microseconds
 * - 3-5× faster ISR execution on 8-bit AVR
 * - Half the memory footprint
 * - Range: 3 ticks (48μs, 20kHz) to 65535 ticks (1.05s, 0.95Hz)
 * 
 * Accumulator Algorithm:
 * - Each ISR adds 8 ticks to each motor's accumulator
 * - When accumulator >= step_interval: generate step, subtract interval
 * - Handles acceleration smoothly: current_speed → target_speed
 * 
 * Supports up to 5 stepper motors with independent speed/direction control.
 */

// Timer constants
constexpr uint8_t ISR_PERIOD_TICKS = 8;      // ISR every 8 ticks (128μs @ 16μs/tick)
constexpr uint16_t TICK_PERIOD_US = 16;      // 16μs per timer tick @ prescaler 256
constexpr uint8_t ACCEL_UPDATE_DIVIDER = 100; // Update acceleration every 100 ISR ticks (11.2ms)
constexpr uint32_t TICKS_PER_SECOND = 62500; // 1000000μs / 16μs/tick

// Static member definitions
volatile uint8_t StepperContinuous::stepper_count_ = 0;
volatile uint8_t StepperContinuous::step_pins_[MAX_STEPPERS] = {0, 0, 0, 0, 0};
volatile uint8_t StepperContinuous::dir_pins_[MAX_STEPPERS] = {0, 0, 0, 0, 0};
volatile int16_t StepperContinuous::current_speeds_[MAX_STEPPERS] = {0, 0, 0, 0, 0};
volatile int16_t StepperContinuous::target_speeds_[MAX_STEPPERS] = {0, 0, 0, 0, 0};

// Accumulator variables (internal to ISR and class methods)
namespace {
    volatile uint16_t step_accumulators[5] = {0, 0, 0, 0, 0};           // Accumulated time in ticks (16-bit for speed!)
    volatile uint16_t step_intervals[5] = {62500, 62500, 62500, 62500, 62500}; // Interval between steps in ticks (1 Hz initial)
    volatile uint16_t step_remainders[5] = {0, 0, 0, 0, 0}; // Remainder of TICKS_PER_SECOND % speed
    volatile uint16_t step_remainder_cntr[5] = {0, 0, 0, 0, 0}; // has vaule from speed to zero; used to redestribute remainder 
    volatile uint8_t accel_counter[5] = {0, 0, 0, 0, 0};                 // Counter for acceleration updates
    volatile bool step_pin_high[5] = {false, false, false, false, false}; // Tracks which pins are HIGH
    volatile uint16_t interval_changes[5] = {5, 5, 5, 5, 5};               // Pre-calculated speed change per accel cycle (avoids ISR division!)
}

// Default constructor
StepperContinuous::StepperContinuous() 
    : step_pin_(0), dir_pin_(0),
      current_speed_sps_(0), target_speed_sps_(0), acceleration_(500),
      motor_index_(255), error_code_(0) {
}

// Constructor with initialization
StepperContinuous::StepperContinuous(uint8_t step_pin, uint8_t dir_pin)
    : step_pin_(step_pin), dir_pin_(dir_pin),
      current_speed_sps_(0), target_speed_sps_(0), acceleration_(500),
      motor_index_(255), error_code_(0) {
    init(step_pin, dir_pin);
}

uint8_t StepperContinuous::init(uint8_t step_pin, uint8_t dir_pin) {
    step_pin_ = step_pin;
    dir_pin_ = dir_pin;
    
    // Find free motor slot
    if (stepper_count_ >= MAX_STEPPERS) {
        error_code_ = 1; // ERROR_NO_FREE_MOTOR
        return error_code_;
    }
    
    motor_index_ = stepper_count_;
    stepper_count_++;
    
    // Configure pins
    //pinMode(step_pin_, OUTPUT);
    //pinMode(dir_pin_, OUTPUT);
    digitalWrite(step_pin_, LOW);
    digitalWrite(dir_pin_, LOW);
    
    // Register motor in static arrays
    uint8_t saved_sreg = SREG;
    cli();
    step_pins_[motor_index_] = step_pin_;
    dir_pins_[motor_index_] = dir_pin_;
    current_speeds_[motor_index_] = 0;
    target_speeds_[motor_index_] = 0;
    step_accumulators[motor_index_] = 0;
    step_intervals[motor_index_] = 62500; // 1 Hz (very slow, essentially stopped)
    SREG = saved_sreg;
    
    // Initialize timer if not already done
    if (!StepCTimerControl::isInitialized()) {
        StepCTimerControl::Timer1_Init();
    }
    
    // Pre-calculate acceleration rate for this motor
    updateAccelerationRate();
    
    error_code_ = 0;
    return 0;
}

void StepperContinuous::setTargetSpeed(int16_t steps_per_sec) {
    // Clamp to valid range: -5000 to +5000 steps/s
    if (steps_per_sec > 5000) steps_per_sec = 5000;
    if (steps_per_sec < -5000) steps_per_sec = -5000;
    
    cli();
    target_speeds_[motor_index_] = steps_per_sec;
    sei();
    
    // Update direction immediately
    updateDirection();
}

void StepperContinuous::setAcceleration(uint16_t steps_per_sec2) {
    acceleration_ = steps_per_sec2;
    updateAccelerationRate();
}

void StepperContinuous::setImmediateSpeed(int16_t steps_per_sec) {
    // Clamp to valid range: -5000 to +5000 steps/s
    if (steps_per_sec > 5000) steps_per_sec = 5000;
    if (steps_per_sec < -5000) steps_per_sec = -5000;
    
    cli();
    current_speeds_[motor_index_] = steps_per_sec;
    target_speeds_[motor_index_] = steps_per_sec;
    sei();
    
    updateDirection();
    updateStepInterval();
}

int16_t StepperContinuous::getCurrentSpeed() {
    return current_speeds_[motor_index_];
}

int16_t StepperContinuous::getTargetSpeed() {
    return target_speeds_[motor_index_];
}

bool StepperContinuous::isMoving() {
    return current_speeds_[motor_index_] != 0;
}


void StepperContinuous::updateStepInterval() {
    int16_t speed = current_speeds_[motor_index_];
    
    if (speed == 0) {
        cli();
        step_intervals[motor_index_] = 62500; // Very slow (1 Hz)
        sei();
        return;
    }
    
    // Calculate interval in ticks (16μs per tick)
    // interval_ticks = 62500 / steps_per_sec
    uint16_t abs_speed = (speed < 0) ? -speed : speed;
    
    uint16_t interval_ticks = TICKS_PER_SECOND / abs_speed;
    uint16_t remainder = TICKS_PER_SECOND % speed;
    if (remainder > 0) {
        interval_ticks++;
    }
    if (interval_ticks < 3) interval_ticks = 3; // Safety: min 3 ticks (48μs, ~20kHz max)
    
    cli();
    step_intervals[motor_index_] = interval_ticks;
    step_remainders[motor_index_] = remainder;
    step_remainder_cntr[motor_index_] = speed;
    sei();
}

void StepperContinuous::updateDirection() {
    int16_t speed = target_speeds_[motor_index_];
    
    if (speed >= 0) {
        digitalWrite(dir_pin_, HIGH);  // Clockwise
    } else {
        digitalWrite(dir_pin_, LOW);   // Counter-clockwise
    }
}

void StepperContinuous::updateAccelerationRate() {
    // Pre-calculate speed change per acceleration cycle (11.2ms)
    // This avoids expensive multiplication in ISR!
    // speed_change = acceleration × 0.0112s
    //              = acceleration × 112 / 10000
    
    uint32_t accel = acceleration_;
    uint16_t speed_change = (accel * 112) / 10000;
    
    if (speed_change == 0) speed_change = 1; // Minimum change of 1 step/s
    
    uint8_t saved_sreg = SREG;
    cli();
    interval_changes[motor_index_] = speed_change;
    SREG = saved_sreg;
}

/**
 * @brief Timer1 ISR for stepper motor control (accumulator approach)
 * 
 * Called every 112μs (8928 Hz) by Timer1 Compare Match A.
 * 
 * Step Pulse Generation:
 * - When step is needed: set pin HIGH and mark it in step_pin_high[]
 * - Next ISR: clear all marked pins to LOW
 * - Result: pulse width = 112μs (hardware needs only ~1μs minimum)
 * 
 * Accumulator Algorithm (16-bit ticks for speed!):
 * 1. Clear all pins that were set HIGH in previous ISR
 * 2. For each active motor:
 *    - Handle acceleration: adjust current_speed towards target_speed (every 100 ISR = 11.2ms)
 *    - Increment accumulator by ISR_PERIOD_TICKS (7 ticks = 112μs)
 *    - While accumulator >= step_interval:
 *      * Set step pin HIGH and mark it
 *      * Subtract step_interval from accumulator
 * 
 * Example (1000 RPM, 200 steps/rev):
 * - Steps/s = 1000 × 200 / 60 = 3333 steps/s
 * - step_interval = 62500 / 3333 = 18.75 ticks ≈ 19 ticks (304μs)
 * - Each ISR (7 ticks = 112μs):
 *   * accumulator += 7 ticks
 *   * If accumulator >= 19 ticks: set pin HIGH, accumulator -= 19 ticks
 *   * Every ~3rd ISR generates one step
 */
void OnTimer1StepperContinuousISR() {
    // First: clear all pins that were set HIGH in previous ISR
    for (uint8_t i = 0; i < StepperContinuous::stepper_count_; i++) {
        if (step_pin_high[i]) {
            digitalWrite(StepperContinuous::step_pins_[i], LOW);
            step_pin_high[i] = false;
        }
    }
    
    // Process each active stepper motor
    for (uint8_t i = 0; i < StepperContinuous::stepper_count_; i++) {
        // Skip if motor not configured
        if (StepperContinuous::step_pins_[i] == 0) continue;
        
        // Handle acceleration/deceleration (only every 100 ISR ticks = ~11.2ms)
        accel_counter[i]++;
        if (accel_counter[i] >= ACCEL_UPDATE_DIVIDER) {
            accel_counter[i] = 0;
            
            int16_t current = StepperContinuous::current_speeds_[i];
            int16_t target = StepperContinuous::target_speeds_[i];
            
            if (current != target) {
                // Use pre-calculated speed change (NO division in ISR!)
                uint16_t speed_change = interval_changes[i];
                
                // Move towards target
                if (current < target) {
                    current += speed_change;
                    if (current > target) current = target;
                } else {
                    current -= speed_change;
                    if (current < target) current = target;
                }
                
                StepperContinuous::current_speeds_[i] = current;
                
                // Recalculate step interval (simplified - direct steps/s to ticks)
                if (current == 0) {
                    step_intervals[i] = 62500; // Stopped (1 Hz)
                } else {
                    uint16_t abs_speed = (current < 0) ? -current : current;
                    if (abs_speed == 0) abs_speed = 1;
                    uint16_t interval = TICKS_PER_SECOND / abs_speed;
                    if (interval < 3) interval = 3; // Safety min
                    step_intervals[i] = interval;
                }
            }
        }
        
        // Skip pulse generation if motor is stopped
        if (StepperContinuous::current_speeds_[i] == 0) continue;
        
        // Accumulator logic for step generation (16-bit arithmetic = fast!)
        step_accumulators[i] += ISR_PERIOD_TICKS; // Add 7 ticks (112μs)
        



        // Generate steps if accumulated time exceeds interval
        while (step_accumulators[i] >= step_intervals[i]) {
            // Set step pin HIGH (will be cleared in next ISR)
            digitalWrite(StepperContinuous::step_pins_[i], HIGH);
            step_pin_high[i] = true; 
            
            // Subtract interval from accumulator
            step_accumulators[i] -= step_intervals[i];
            
            //division causes that ticks are not precise, so add a litle delay to some
            //ie.. if i have speedX ticks per second. add +1 for first step_remainers ticks. 
            //examle:: speed 3000ticks per sec. remainder 2500. add extra delau for first 2500 of 3000 ticks
            if (StepperContinuous::current_speeds_[i] > 0 && step_remainders[i] > 0)
            {
                if (step_remainder_cntr[i]  > step_remainders[i]){
                    step_accumulators[i] += 1;
                }       
                step_remainder_cntr[i]--;
                if (step_remainder_cntr[i] == 0) { 
                    step_remainder_cntr[i] = StepperContinuous::current_speeds_[i];
                }
            }
        }
    }
}


