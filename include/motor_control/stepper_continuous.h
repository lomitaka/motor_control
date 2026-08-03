/**
 * @file stepper_continuous.h
 * @brief Stepper motor driver for continuous rotation (steps/s control)
 * 
 * Controls stepper motor with continuous rotation at specified speed.
 * Suitable for applications like conveyor belts, fans, or constant speed movement.
 * 
 * @example
 * StepperContinuous motor(2, 3);       // STEP=pin2, DIR=pin3
 * motor.setAcceleration(500);          // 500 steps/s² acceleration
 * motor.setTargetSpeed(1000);          // 1000 steps/s clockwise
 * motor.setTargetSpeed(-500);          // 500 steps/s counter-clockwise
 * motor.setImmediateSpeed(0);          // Emergency stop (no deceleration)
 */

#ifndef STEPPER_CONTINUOUS_H
#define STEPPER_CONTINUOUS_H

#include <stdint.h>
#include "motor.h"

/**
 * @brief Stepper motor controller for continuous rotation
 * 
 * Features:
 * - Speed control in steps per second
 * - Configurable acceleration/deceleration ramps
 * - Direction control (positive=CW, negative=CCW)
 * - Emergency stop capability
 */
class StepperContinuous {
public:
    /**
     * @brief Default constructor (must call init())
     */
    StepperContinuous();
    
    /**
     * @brief Construct and initialize stepper motor
     * @param step_pin Arduino pin number for STEP signal
     * @param dir_pin Arduino pin number for DIR (direction) signal
     */
    StepperContinuous(uint8_t step_pin, uint8_t dir_pin);
    
    /**
     * @brief Initialize stepper motor pins
     * @param step_pin Arduino pin number for STEP signal
     * @param dir_pin Arduino pin number for DIR signal
     * @return 0 on success, error code otherwise
     */
    uint8_t init(uint8_t step_pin, uint8_t dir_pin);
    
    /**
     * @brief Set target speed with smooth acceleration
     * @param steps_per_sec Target speed in steps per second
     *                      - Range: -5000 to +5000 steps/s (clamped automatically)
     *                      - Positive values: clockwise rotation
     *                      - Negative values: counter-clockwise rotation
     *                      - Zero: smooth stop (with deceleration)
     * 
     * Motor will accelerate/decelerate to target speed using configured acceleration.
     */
    void setTargetSpeed(int16_t steps_per_sec);
    
    /**
     * @brief Set acceleration/deceleration rate
     * @param steps_per_sec2 Acceleration in steps per second squared
     *                        Typical values: 100-1000 for smooth operation
     *                        Higher = faster speed changes, but may cause skipped steps
     */
    void setAcceleration(uint16_t steps_per_sec2);
    
    /**
     * @brief Set speed immediately without acceleration ramp
     * @param steps_per_sec Target speed in steps/s
     *                      - Range: -5000 to +5000 steps/s (clamped automatically)
     *                      - Positive=CW, negative=CCW
     * 
     * WARNING: Instant speed changes may cause:
     * - Skipped steps at high speeds
     * - Mechanical stress
     * - Loss of position accuracy
     * 
     * Use for emergency stops or when starting from stationary position.
     */
    void setImmediateSpeed(int16_t steps_per_sec);
    
    /**
     * @brief Get current speed
     * @return Current speed in steps/s (positive=CW, negative=CCW)
     */
    int16_t getCurrentSpeed();
    
    /**
     * @brief Get target speed
     * @return Target speed in steps/s
     */
    int16_t getTargetSpeed();
    
    /**
     * @brief Check if motor is currently moving
     * @return true if speed > 0, false if stopped
     */
    bool isMoving();
    
    /**
     * @brief Get last error code
     * @return Error code (0 = no error)
     */
    uint8_t getLastError();

private:
    // Pin configuration
    uint8_t step_pin_;           ///< STEP signal pin
    uint8_t dir_pin_;            ///< Direction signal pin
    
    // Speed control
    int16_t current_speed_sps_;  ///< Current speed in steps/s
    int16_t target_speed_sps_;   ///< Target speed in steps/s
    uint16_t acceleration_;      ///< Steps per second squared
    
    // Status
    uint8_t motor_index_;        ///< Index in internal motor arrays
    uint8_t error_code_;         ///< Last error code
    
    // Internal helper methods
    void updateStepInterval();     ///< Calculate step interval from current speed
    void updateDirection();        ///< Set direction pin based on speed sign
    void updateAccelerationRate(); ///< Pre-calculate speed change for acceleration (avoids division in ISR)
    
    // Static arrays for managing multiple steppers (shared with Timer ISR)
    static constexpr uint8_t MAX_STEPPERS = 5;
    
    volatile static uint8_t stepper_count_;
    volatile static uint8_t step_pins_[MAX_STEPPERS];
    volatile static uint8_t dir_pins_[MAX_STEPPERS];
    volatile static int16_t current_speeds_[MAX_STEPPERS];
    volatile static int16_t target_speeds_[MAX_STEPPERS];
    
    friend void OnTimer1StepperISR();
};

#endif // STEPPER_CONTINUOUS_H
