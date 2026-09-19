/**
 * @file stepper_positioning.h
 * @brief Stepper motor driver for precise positioning (step-based control)
 * 
 * Controls stepper motor for precise positioning with acceleration/deceleration.
 * Suitable for robotic arms, 3D printers, CNC machines, or any application
 * requiring accurate position control.
 * 
 * Architecture:
 * - Uses Timer1 without prescaler (16 MHz, 62.5ns tick)
 * - OCR1A dynamically scheduled for next motor event (like DC control)
 * - OCR1B scheduled for falling edges (few ticks after OCR1A)
 * - Binary mask tracks which motors need falling edge
 * - Tracks remaining steps and automatically calculates braking distance
 * 
 * @example
 * StepperPositioning motor(2, 3);       // STEP=pin2, DIR=pin3
 * motor.setAcceleration(500);           // 500 steps/s² acceleration
 * motor.setSpeed(1000);                 // Maximum speed: 1000 steps/s
 * motor.setTargetTicks(200);            // Move 200 steps with acceleration
 * motor.addTargetTicks(-100);           // Move 100 steps back
 * motor.setImmediateTicks(50);          // Instant 50 steps (no accel)
 * 
 * while(motor.isMoving()) {
 *     // Wait for movement to complete
 * }
 */

#ifndef STEPPER_POSITIONING_H
#define STEPPER_POSITIONING_H

#include <stdint.h>
#include "debug.h"

/**
 * @brief Stepper motor controller for precise positioning
 * 
 * Features:
 * - Step-based positioning (direct step count control)
 * - Configurable acceleration/deceleration profiles
 * - Trapezoidal motion profile (accel → constant → decel)
 * - Automatic braking distance calculation
 * - Position tracking (remaining steps)
 * - Movement completion detection
 * - Up to 5 motors controlled simultaneously
 */
class StepperPositioning {
public:
    /**
     * @brief Default constructor (must call init())
     */
    StepperPositioning();
    
    /**
     * @brief Construct and initialize stepper motor
     * @param step_pin Arduino pin number for STEP signal
     * @param dir_pin Arduino pin number for DIR (direction) signal
     */
    StepperPositioning(uint8_t step_pin, uint8_t dir_pin);
    
    /**
     * @brief Initialize stepper motor pins
     * @param step_pin Arduino pin number for STEP signal
     * @param dir_pin Arduino pin number for DIR signal
     * @return 0 on success, error code otherwise
     */
    uint8_t init(uint8_t step_pin, uint8_t dir_pin);
    

    /*
      Sets initial ticking speed for the moment when motor is spining up. 
      if is set speed that is slower than steps_per_sec, it is considered as stop speed.
    */
    void setSpinupSpeedTicks(uint16_t steps_per_sec);


    /**
     * @brief Set target amount of ticks to perform
     * @param steps          Number of steps to move
     *                      - Positive values: clockwise rotation
     *                      - Negative values: counter-clockwise rotation
     *                      - Zero: smooth stop (with deceleration)
     * 
     * Motor will accelerate to target speed, then decelerate to stop at target position.
     * If the requested steps are less than the braking distance needed to stop,
     * the command is rejected and returns error code 1.
     */
     void setTargetTicks(int16_t steps);

/**
     * @brief Set target speed for positioning movements
     * @param steps_per_sec  Target speed in steps per second
     *                      - Range: -5000 to +5000 steps/s (clamped automatically)
     *                      - This speed will be used during constant-speed phase of movement
     * @return              0 on success
     * 
     * Does not start motor movement, just sets the speed motor should reach during
     * positioning. Automatically recalculates braking distance based on this speed.
     * Initial value: 0 steps/s (motor stopped)
     * Note: any speed that is slower than spinupSpeed is considered as stop speed.
     */
    uint8_t setSpeed(uint16_t steps_per_sec);

    /**
     * @brief Add steps to current target position
     * @param steps         Steps to add to current target (can be negative)
     * 
     * Modifies the current remaining tick count. May fail if the resulting
     * target position would require more braking distance than available.
     */
    void addTargetTicks(int16_t steps);
    
    /**
     * @brief Set acceleration/deceleration rate
     * @param percent_increase speed change in percents. (valid values 1-5)
     * 
     * Automatically recalculates braking distance when changed.
     */
    void setAcceleration(uint8_t percent_increase);
    
    /**
     * @brief Set steps immediately without acceleration ramp
     * @param steps         Number of steps to perform
     *                      - Range: -5000 to +5000 steps (clamped automatically)
     *                      - Positive=CW, negative=CCW
     * 
     * WARNING: Instant step commands may cause:
     * - Skipped steps if motor can't respond fast enough
     * - Mechanical stress
     * - Loss of position accuracy
     * 
     * Use for emergency stops or when starting from stationary position.
     */
    void setImmediateTicks(int16_t steps);
    
    /**
     * @brief Check if motor is currently moving
     * @return true if motor has remaining steps or non-zero speed, false if stopped
     */
    bool isMoving();



private:
    // Pin configuration
    uint8_t step_pin_;           ///< STEP signal pin
    uint8_t dir_pin_;            ///< Direction signal pin
    
    // Speed control
    int16_t current_speed_sps_;  ///< Current speed in steps/s
    int16_t target_speed_sps_;   ///< Target speed in steps/s
    
    // Status
    uint8_t motor_index_;        ///< Index in internal motor arrays
    
    // Internal helper methods
    void updateStepInterval();     ///< Calculate step interval from target speed
    void updateDirection();        ///< Set direction pin based on speed sign
    void updateAccelerationRate(); ///< Pre-calculate speed change for acceleration (avoids division in ISR)
    void updateBrakingDistance();  ///< Calculate braking distance based on current speed and acceleration
    
    // Static arrays for managing multiple steppers (shared with Timer ISR)
    static constexpr uint8_t MAX_STEPPERS = 5;
    
    volatile static uint8_t stepper_count_;
    volatile static uint8_t step_pins_[MAX_STEPPERS];
    volatile static uint8_t dir_pins_[MAX_STEPPERS];
    volatile static int16_t current_speeds_[MAX_STEPPERS];
    volatile static uint16_t target_speeds_stp_ps_[MAX_STEPPERS];
    
    friend void OnTimer1StepperPositioningOverflow();
    friend void OnTimer1StepperPositioningOCRA();
    friend void OnTimer1StepperPositioningOCRB();
    friend DebugInfo GetDebugInfo();
};

#endif // STEPPER_POSITIONING_H
