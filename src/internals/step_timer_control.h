#ifndef STEP_TIMER_CONTROL_H
#define STEP_TIMER_CONTROL_H

#ifdef SIMULATION_MODE
    #include "simulator/avr_mock.h"
#else
    #include <avr/io.h>
#endif

#include <stdint.h>

/**
 * @brief Timer1 control for stepper motors with prescaler 256
 * 
 * Timer Configuration:
 * - Prescaler: 256 → Timer freq = 16 MHz / 256 = 62.5 kHz
 * - Tick period: 16 μs
 * - CTC mode with OCR1A = 7 → 7 × 16μs = 112 μs period (8928 Hz ISR)
 * - Max speed: ~10000 steps/s per motor (allows ~11 ticks per step)
 * - Good resolution for 0-3000 RPM range
 */
class StepTimerControl {
public:
    /**
     * @brief Initialize Timer1 for stepper motor control
     * 
     * Sets up Timer1 in CTC mode with:
     * - Prescaler 256 (CS12=1)
     * - OCR1A = 625 (10ms period, 100 Hz)
     * - Compare Match A interrupt enabled
     */
    static void Timer1_Init();
    
    /**
     * @brief Check if timer is initialized
     */
    static bool isInitialized();
    
private:
    static bool initialized_;
};

// Forward declaration for ISR
void OnTimer1StepperISR();

#endif // STEP_TIMER_CONTROL_H
