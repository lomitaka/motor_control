#ifndef STEPP_TIMER_CONTROL_H
#define STEPP_TIMER_CONTROL_H

#ifdef SIMULATION_MODE
    #include "simulator/avr_mock.h"
#else
    #include <avr/io.h>
#endif

#include <stdint.h>

/**
 * @brief Timer1 control for stepper positioning motors (no prescaler)
 * 
 * Timer Configuration:
 * - Prescaler: 1 (no prescaler) → Timer freq = 16 MHz
 * - Tick period: 62.5 ns
 * - Normal mode: Timer counts 0→65535, then overflows
 * - Overflow period: 4.096 ms (used for acceleration updates and overflow counting)
 * - OCR1A: Dynamically scheduled for next motor event
 * - OCR1B: Scheduled few ticks after OCR1A for falling edges
 * 
 * Architecture:
 * - Multiple steppers (up to 5) share one timer
 * - Overflow interrupt: Decrement overflow_skip_count, handle acceleration
 * - OCRA interrupt: Generate STEP rising edges when overflow_skip_count == 0
 * - OCRB interrupt: Generate STEP falling edges (cleanup)
 * - Uses 16-bit arithmetic only (overflow_skip_count + tick_count) for speed
 */
class SteppTimerControl {
public:
    /**
     * @brief Initialize Timer1 for stepper positioning control
     * 
     * Sets up Timer1 in Normal mode with:
     * - No prescaler (CS10=1) → 16 MHz
     * - Overflow every 65536 ticks = 4.096 ms
     * - OCR1A = dynamic (next motor event)
     * - OCR1B = OCR1A + edge delay (for falling edge)
     * - Compare Match A, B, and Overflow interrupts enabled
     */
    static void Timer1_Init();
    
    /**
     * @brief Check if timer is initialized
     */
    static bool isInitialized();
    
private:
    static bool initialized_;
};

// Forward declarations for ISRs
void OnTimer1StepperPositioningOverflow();
void OnTimer1StepperPositioningOCRA();
void OnTimer1StepperPositioningOCRB();

#endif // STEPP_TIMER_CONTROL_H
