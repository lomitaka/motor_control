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
 * - CTC mode with OCR1A as dynamic TOP (WGM12)
 * - OCR1A: Dynamically scheduled for next motor event (like DC control)
 * - OCR1B: Scheduled few ticks after OCR1A for falling edges
 * 
 * Architecture (similar to DC control):
 * - Multiple steppers (up to 5) share one timer
 * - OCRA interrupt: Generate STEP rising edges, schedule next event
 * - OCRB interrupt: Generate STEP falling edges (cleanup)
 * - Binary array tracks which motors need falling edge
 */
class SteppTimerControl {
public:
    /**
     * @brief Initialize Timer1 for stepper positioning control
     * 
     * Sets up Timer1 in CTC mode with:
     * - No prescaler (CS10=1) → 16 MHz
     * - OCR1A = dynamic (next motor event)
     * - OCR1B = OCR1A + edge delay (for falling edge)
     * - Compare Match A and B interrupts enabled
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
void OnTimer1StepperPositioningOCRA();
void OnTimer1StepperPositioningOCRB();

#endif // STEPP_TIMER_CONTROL_H
