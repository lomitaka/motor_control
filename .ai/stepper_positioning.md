# Stepper Positioning Control - Technická Dokumentace

## Přehled
Implementace řízení krokových motorů pro přesné polohování s podporou až 5 motorů současně na AVR ATmega328p.

## Soubory
- **Header**: `include/motor_control/stepper_positioning.h`
- **Implementace**: `src/stepp_control.cpp`
- **Timer control**: `src/internals/stepp_timer_control.h`, `src/stepp_timer_control.cpp`

## Hardware - Timer1 Konfigurace

### Základní parametry
- **CPU frekvence**: 16 MHz
- **Timer**: Timer1 (16-bit)
- **Režim**: CTC (Clear Timer on Compare Match A)
- **Prescaler**: 1 (no prescaler - full speed)
- **OCR1A**: Dynamický (next motor event)
- **OCR1B**: Dynamický (OCR1A + STEP_PULSE_TICKS)

### Výpočet timingu
```
Timer frequency = 16 MHz / 1 = 16 MHz
Tick period = 62.5 ns
Timer range = 0-65535 ticks = 0-4.096 ms
```

### Konstanty
```cpp
constexpr uint16_t TIMER_GUARD_TICKS = 100;     // 6.25 μs safety margin
constexpr uint16_t STEP_PULSE_TICKS = 20;       // 1.25 μs pulse width
constexpr uint32_t TICKS_PER_SECOND = 16000000; // 16 MHz
```

### Registry setup
```cpp
TCCR1A = 0;                          // Normal port operation
TCCR1B = (1<<WGM12) | (1<<CS10);    // CTC mode, no prescaler
OCR1A = 64000;                       // Initial (updated dynamically)
OCR1B = 65535;                       // Initial (updated dynamically)
TIMSK1 = (1<<OCIE1A) | (1<<OCIE1B); // Enable both compare interrupts
```

## Architektura - Dynamic Event Scheduling

### Hlavní koncept
Podobně jako DC control, používá **dynamické plánování** místo fixního ISR periodu:

- **OCR1A**: Naplánováno na čas dalšího kroku nejbližšího motoru
- **OCR1B**: Naplánováno na OCR1A + STEP_PULSE_TICKS pro falling edge
- **Event-driven**: ISR se volá pouze když nějaký motor potřebuje step

### Scheduler Princip
```
Motor A: Next step at t=1000 ticks
Motor B: Next step at t=1500 ticks
Motor C: Next step at t=2000 ticks

Timeline:
t=1000: OCRA ISR → Motor A step HIGH, schedule OCRB=1020
t=1020: OCRB ISR → Motor A step LOW
t=1500: OCRA ISR → Motor B step HIGH, schedule OCRB=1520
t=1520: OCRB ISR → Motor B step LOW
...and so on
```

**Výhoda:** CPU load pouze když je potřeba, ne na fixní frekvenci.

## Positioning Logic

### Position Tracking
Každý motor sleduje:
```cpp
volatile int16_t remaining_ticks[5];      // Kolik kroků zbývá do cíle
volatile int16_t braking_distance[5];     // Kolik kroků na zastavení
volatile uint32_t next_step_time[5];      // Absolutní čas příštího kroku
volatile uint16_t step_interval_ticks[5]; // Ticks mezi kroky
```

### Trapezoidal Motion Profile
```
Speed
  ^
  |     /‾‾‾‾‾\         Constant speed phase
  |    /       \
  |   /         \       Deceleration
  |  /           \
  | /Acceleration \
  +-----------------> Time/Steps
  0   A    B   C  D

Phases:
A: Acceleration (speed increases)
B: Constant speed (target speed reached)
C: Deceleration starts (braking_distance reached)
D: Stop (remaining_ticks = 0)
```

### Braking Distance Calculation
Vypočítáno z kinematické rovnice: **v² = v₀² + 2as**

Řešením pro **s**: **s = v² / (2a)**

```cpp
void updateBrakingDistance() {
    uint16_t abs_speed = (speed < 0) ? -speed : speed;
    uint32_t speed_squared = (uint32_t)abs_speed * abs_speed;
    uint16_t brake_dist = speed_squared / (2 * acceleration);
    braking_distance[motor_index_] = brake_dist;
}
```

**Příklad:**
- Speed: 1000 steps/s
- Acceleration: 500 steps/s²
- Braking distance = 1000² / (2 × 500) = 1000 kroků

### Auto-braking
ISR automaticky spouští braking:
```cpp
int16_t abs_remaining = (remaining < 0) ? -remaining : remaining;
if (abs_remaining <= brake_dist && remaining != 0) {
    target = 0;  // Start deceleration
}
```

## API

### 1. setTargetTicks(int16_t steps)
Nastaví cílový počet kroků k provedení.

```cpp
motor.setSpeed(1000);           // 1000 steps/s max speed
motor.setTargetTicks(500);      // Move 500 steps forward
// Returns 0 on success, 1 if steps < braking_distance
```

**Validace:**
```cpp
if (abs_steps < brake_dist) {
    return 1; // ERROR: Cannot execute with current deceleration
}
```

### 2. setSpeed(int16_t steps_per_sec)
Nastaví maximální rychlost pro pohyby.

```cpp
motor.setSpeed(2000);  // 2000 steps/s
// Automatically recalculates braking_distance
```

**Range:** -5000 to +5000 steps/s (clamped)

### 3. addTargetTicks(int16_t steps)
Přidá kroky k aktuálnímu cíli.

```cpp
motor.setTargetTicks(1000);   // Move to +1000
motor.addTargetTicks(-200);   // Now target is +800
```

### 4. setImmediateTicks(int16_t steps)
Okamžité nastavení kroků bez akcelerace.

```cpp
motor.setImmediateTicks(100);  // Instant 100 steps
// WARNING: May cause skipped steps!
```

### 5. isMoving()
Zkontroluje, zda se motor pohybuje.

```cpp
while(motor.isMoving()) {
    // Wait for completion
}
```

## ISR Struktura

### OCRA ISR (Rising Edges + Scheduling)

```cpp
void OnTimer1StepperPositioningOCRA() {
    uint16_t current_time = TCNT1;
    uint32_t next_event = UINT32_MAX;
    
    // 1. Process each motor
    for (uint8_t i = 0; i < stepper_count; i++) {
        // Check if this motor needs step now
        if (next_step_time[i] <= current_time + GUARD) {
            // Generate step
            if (remaining_ticks[i] != 0) {
                digitalWrite(step_pins_[i], HIGH);
                step_pins_high_mask |= (1 << i);
                
                // Update position
                remaining_ticks[i] += (remaining_ticks[i] > 0) ? -1 : 1;
                
                // Schedule next step for this motor
                next_step_time[i] = current_time + step_interval_ticks[i];
            }
            
            // Handle acceleration/deceleration
            accelerate_motor(i);
        }
        
        // Find nearest event across all motors
        if (remaining_ticks[i] != 0 && next_step_time[i] < next_event) {
            next_event = next_step_time[i];
        }
    }
    
    // 2. Schedule next OCRA
    OCR1A = calculate_safe_compare(next_event);
    
    // 3. Schedule OCRB for falling edges
    if (step_pins_high_mask != 0) {
        OCR1B = current_time + STEP_PULSE_TICKS;
    }
}
```

### OCRB ISR (Falling Edges)

```cpp
void OnTimer1StepperPositioningOCRB() {
    // Clear all pins that were set HIGH
    for (uint8_t i = 0; i < stepper_count; i++) {
        if (step_pins_high_mask & (1 << i)) {
            digitalWrite(step_pins_[i], LOW);
        }
    }
    step_pins_high_mask = 0;
}
```

**Pulse timing:**
```
        ┌─────┐
STEP ───┘     └─────
        │←1.25μs→│
        OCRA   OCRB
```

## Optimalizace

### 1. Binary Mask pro HIGH pins
Místo pole `bool[5]` používá `uint8_t` bitmask:
```cpp
volatile uint8_t step_pins_high_mask = 0;  // 1 byte vs 5 bytes

// Set HIGH
step_pins_high_mask |= (1 << motor_index);

// Check
if (step_pins_high_mask & (1 << i)) { ... }
```

### 2. Guard Time
Zabraňuje missed compare match během ISR:
```cpp
uint16_t safe_next = next_event - current_time;
if (safe_next < TIMER_GUARD_TICKS) {
    safe_next = TIMER_GUARD_TICKS;
}
OCR1A = current_time + safe_next;
```

### 3. Pre-calculated Intervals
Step interval se počítá mimo ISR:
```cpp
// In setSpeed():
uint32_t interval = TICKS_PER_SECOND / abs_speed;
step_interval_ticks[motor_index_] = (uint16_t)interval;

// In ISR: just load
next_step_time[i] += step_interval_ticks[i];  // Fast!
```

## Performance

### CPU Load Odhad
Předpoklad: 5 motorů @ 1000 steps/s

```
OCRA ISR:
- Loop: 5 × ~50 CPU cycles = 250 cycles
- Scheduling: ~30 cycles
- Total: ~300 cycles @ 16 MHz = 18.75 μs

OCRB ISR:
- Loop: 5 × ~20 cycles = 100 cycles
- Total: ~100 cycles = 6.25 μs

Per step: 300 + 100 = 400 cycles = 25 μs
With 5 motors @ 1000 Hz each: 5000 steps/s total
CPU time: 5000 × 25 μs = 125 ms/s = 12.5% CPU load
```

**Výsledek:** ~10-15% CPU @ plném zatížení (5 motorů, plná rychlost)

## Kompatibilita

### Timer Konflikty
⚠️ **NELZE kombinovat:**
- DC/Servo Control (Timer1, no prescaler, fixed CTC @ 64000)
- Stepper Continuous (Timer1, prescaler 256, fixed CTC @ 7)
- **Stepper Positioning (Timer1, no prescaler, dynamic CTC)**

**Důvod:** Všechny tři systémy rekonfigurují Timer1 různě.

### Doporučené kombinace
✅ **Možné:**
- Stepper Positioning + Timer0/Timer2 pro jiné účely
- Stepper Positioning s jinými periferiemi (UART, SPI, I2C)

❌ **Nemožné:**
- Stepper Positioning + DC Control současně
- Stepper Positioning + Stepper Continuous současně
- Stepper Positioning + Servo Control současně

## Příklad použití

### Základní positioning
```cpp
#include "motor_control/stepper_positioning.h"

StepperPositioning motor(2, 3);  // STEP=pin2, DIR=pin3

motor.setAcceleration(500);      // 500 steps/s² accel
motor.setSpeed(1000);            // 1000 steps/s max speed

// Move 2000 steps with acceleration
motor.setTargetTicks(2000);

// Wait for completion
while(motor.isMoving()) {
    // Your code here
}

// Move back
motor.setTargetTicks(-1000);
while(motor.isMoving());
```

### Multi-motor coordinated motion
```cpp
StepperPositioning motor1(2, 3);
StepperPositioning motor2(4, 5);

motor1.setSpeed(1000);
motor2.setSpeed(1500);

// Start both motors simultaneously
motor1.setTargetTicks(1000);
motor2.setTargetTicks(1500);

// Wait for both to complete
while(motor1.isMoving() || motor2.isMoving());
```

### Error handling
```cpp
motor.setSpeed(2000);
motor.setAcceleration(100);  // Low acceleration

uint8_t result = motor.setTargetTicks(50);
if (result != 0) {
    // Error: Not enough distance to brake!
    // Need to increase acceleration or distance
    motor.setAcceleration(500);
    result = motor.setTargetTicks(50);
}
```

## Debugging

### Common Issues

**1. Motor doesn't move**
- Check: `isMoving()` returns true?
- Check: `setSpeed()` called before `setTargetTicks()`?
- Check: Pins configured correctly?

**2. Steps < braking_distance error**
- Solution: Increase acceleration
- Solution: Increase target steps
- Solution: Reduce max speed

**3. Jerky movement**
- Check: Acceleration too high?
- Check: Speed too high for motor?
- Solution: Reduce acceleration/speed

**4. Lost steps**
- Check: Using `setImmediateTicks()` at high speed?
- Solution: Use `setTargetTicks()` with acceleration
- Check: Power supply adequate?

## Future Improvements

### 1. S-curve acceleration
Místo lineární rampy použít S-křivku pro plynulejší pohyb:
```
Linear:    |  /
           | /
           |/

S-curve:   |    ___
           |  /
           | /
           |/
```

### 2. Time-based acceleration
Aktuálně: Akcelerace per-step (nepřesné při rychlých změnách)
Lepší: Akcelerace per-time (přesnější fyzikální model)

### 3. Trajectory pre-calculation
Předvýpočet celé trajektorie před začátkem pohybu:
- Přesný čas arrival
- Garantovaná akcelerace
- Možnost optimalizace více motorů

### 4. Position feedback
Přidat support pro enkodéry:
- Closed-loop control
- Detekce ztracených kroků
- Auto-korekce pozice

---

## Srovnání: Positioning vs Continuous

| Vlastnost | Positioning | Continuous |
|-----------|-------------|------------|
| **Timer prescaler** | 1 (16 MHz) | 256 (62.5 kHz) |
| **ISR trigger** | Dynamic (event-driven) | Fixed (every 112 μs) |
| **Position tracking** | ✅ Yes (remaining_ticks) | ❌ No |
| **Braking distance** | ✅ Auto-calculated | ❌ Manual |
| **API** | Step-based | Speed-based |
| **Use case** | Precise positioning | Continuous rotation |
| **CPU load** | ~10-15% @ full | ~25% @ full |
| **Accuracy** | High (event-driven) | Good (accumulator) |
