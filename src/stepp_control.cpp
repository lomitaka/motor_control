
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
volatile int16_t StepperPositioning::target_speeds_[MAX_STEPPERS] = {0, 0, 0, 0, 0};

// Internal ISR variables
namespace {
    // Position tracking
    volatile int16_t remaining_ticks[5] = {0, 0, 0, 0, 0};      // Remaining steps to target
    volatile int16_t braking_distance[5] = {0, 0, 0, 0, 0};    // Steps needed to stop
    
    // Timing
    volatile uint32_t next_step_time[5] = {0, 0, 0, 0, 0};     // Absolute time of next step (in timer ticks)
    volatile uint16_t step_interval_ticks[5] = {16000, 16000, 16000, 16000, 16000}; // Ticks between steps
    
    // Acceleration
    volatile uint16_t acceleration_rates[5] = {500, 500, 500, 500, 500}; // steps/s²
    volatile uint16_t speed_change_per_step[5] = {1, 1, 1, 1, 1};        // Pre-calculated acceleration
    
    // Step pulse tracking
    volatile uint8_t step_pins_high_mask = 0;  // Bitmask: which motors need falling edge
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
    next_step_time[motor_index_] = TCNT1 + 10000; // Start in future
    step_interval_ticks[motor_index_] = 16000;    // 1 kHz initial
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
    // Clamp to valid range
    if (steps > 16000) steps = 16000;
    if (steps < -16000) steps = -16000;
    
    cli();
    int16_t current_remaining = remaining_ticks[motor_index_];
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
    cli();
    int16_t new_target = remaining_ticks[motor_index_] + steps;
    sei();
    
    return setTargetTicks(new_target);
}

void StepperPositioning::setAcceleration(uint16_t steps_per_sec2) {
    acceleration_ = steps_per_sec2;
    cli();
    acceleration_rates[motor_index_] = steps_per_sec2;
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
        step_interval_ticks[motor_index_] = 16000; // Very slow (1 kHz)
        sei();
        return;
    }
    
    // Calculate interval in ticks
    // interval_ticks = TICKS_PER_SECOND / steps_per_sec
    uint16_t abs_speed = (speed < 0) ? -speed : speed;
    if (abs_speed == 0) abs_speed = 1;
    
    uint32_t interval = TICKS_PER_SECOND / abs_speed;
    if (interval > 65535) interval = 65535;
    if (interval < 100) interval = 100; // Safety: min 100 ticks (6.25μs)
    
    cli();
    step_interval_ticks[motor_index_] = (uint16_t)interval;
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
    // Pre-calculate speed change per step to avoid division in ISR
    // This calculates how much the speed changes for each step during acceleration
    
    uint32_t accel = acceleration_;
    if (accel == 0) accel = 1;
    
    // speed_change = acceleration / steps_per_update
    // For simplicity: update every step, so speed_change = accel / current_speed
    // But we'll pre-calculate a reasonable increment
    uint16_t speed_change = 1;
    
    if (accel >= 1000) speed_change = 10;
    else if (accel >= 500) speed_change = 5;
    else if (accel >= 100) speed_change = 2;
    else speed_change = 1;
    
    cli();
    speed_change_per_step[motor_index_] = speed_change;
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
 * @brief Timer1 OCRA ISR - Rising edge generation and next event scheduling
 * 
 * Called when TCNT1 reaches OCR1A. This ISR:
 * 1. Generates STEP rising edges for motors whose next_step_time has arrived
 * 2. Updates motor states (acceleration, position, speed)
 * 3. Schedules next OCR1A for the nearest motor event
 * 4. Schedules OCR1B for falling edges (few ticks later)
 */
void OnTimer1StepperPositioningOCRA() {
    uint16_t current_time = TCNT1;
    uint32_t next_event = current_time + 65535; // Far future
    step_pins_high_mask = 0;
    
    // Process each active motor
    for (uint8_t i = 0; i < StepperPositioning::stepper_count_; i++) {
        // Skip if motor not configured
        if (StepperPositioning::step_pins_[i] == 0) continue;
        
        // Check if this motor needs a step now
        if (next_step_time[i] <= current_time + TIMER_GUARD_TICKS) {
            // Generate step if motor still has remaining ticks
            if (remaining_ticks[i] != 0) {
                // Set step pin HIGH
                digitalWrite(StepperPositioning::step_pins_[i], HIGH);
                step_pins_high_mask |= (1 << i);
                
                // Decrement remaining ticks
                if (remaining_ticks[i] > 0) {
                    remaining_ticks[i]--;
                } else {
                    remaining_ticks[i]++;
                }
                
                // Update next step time
                next_step_time[i] = current_time + step_interval_ticks[i];
            }
            
            // Handle acceleration/deceleration
            int16_t current = StepperPositioning::current_speeds_[i];
            int16_t target = StepperPositioning::target_speeds_[i];
            int16_t remaining = remaining_ticks[i];
            int16_t brake_dist = braking_distance[i];
            
            // Check if we need to start braking
            int16_t abs_remaining = (remaining < 0) ? -remaining : remaining;
            if (abs_remaining <= brake_dist && remaining != 0) {
                // Start deceleration
                target = 0;
            }
            
            // Accelerate/decelerate
            if (current != target) {
                uint16_t change = speed_change_per_step[i];
                
                if (current < target) {
                    current += change;
                    if (current > target) current = target;
                } else {
                    current -= change;
                    if (current < target) current = target;
                }
                
                StepperPositioning::current_speeds_[i] = current;
                
                // Recalculate step interval
                if (current != 0) {
                    uint16_t abs_speed = (current < 0) ? -current : current;
                    if (abs_speed == 0) abs_speed = 1;
                    uint32_t interval = TICKS_PER_SECOND / abs_speed;
                    if (interval > 65535) interval = 65535;
                    if (interval < 100) interval = 100;
                    step_interval_ticks[i] = (uint16_t)interval;
                }
            }
            
            // Stop if reached target
            if (remaining_ticks[i] == 0) {
                StepperPositioning::current_speeds_[i] = 0;
            }
        }
        
        // Find nearest next event
        if (remaining_ticks[i] != 0 && next_step_time[i] < next_event) {
            next_event = next_step_time[i];
        }
    }
    
    // Schedule next OCRA
    if (next_event < current_time + 65535) {
        uint16_t safe_next = (uint16_t)(next_event - current_time);
        if (safe_next < TIMER_GUARD_TICKS) safe_next = TIMER_GUARD_TICKS;
        OCR1A = current_time + safe_next;
    } else {
        OCR1A = current_time + 10000; // Default: check again in 625μs
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
