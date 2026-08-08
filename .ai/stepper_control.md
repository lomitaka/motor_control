# Stepper Motor Control - Technická Dokumentace

## Přehled
Implementace řízení krokových motorů (stepper motors) pro AVR ATmega328p s pokročilou optimalizací pro 8-bitovou architekturu.

## Soubory
- **Header**: `include/motor_control/stepper_continuous.h`
- **Implementace**: `src/step_control.cpp`
- **Timer control**: `src/internals/step_timer_control.h`, `src/step_timer_control.cpp`

## Hardware - Timer1 Konfigurace

### Základní parametry
- **CPU frekvence**: 16 MHz
- **Timer**: Timer1 (16-bit)
- **Režim**: CTC (Clear Timer on Compare Match)
- **Prescaler**: 256
- **OCR1A**: 7 (compare value)

### Výpočet timingu
```
Timer frequency = 16 MHz / 256 = 62.5 kHz
Tick period = 16 μs
ISR period = 7 ticks × 16 μs = 112 μs
ISR frequency = 8928 Hz (~9 kHz)
```

### Registry setup
```cpp
TCCR1A = 0;                          // Normal port operation
TCCR1B = (1<<WGM12) | (1<<CS12);    // CTC mode, prescaler 256
OCR1A = 7;                           // Compare value
TIMSK1 = (1<<OCIE1A);                // Enable compare interrupt
```

## Architektura

### Akumulátorový přístup (Accumulator Approach)
Místo generování přesných delay mezi kroky používáme akumulátor:

```
Každý ISR (112 μs):
  accumulator += 7 ticks
  if (accumulator >= step_interval):
      generate_step()
      accumulator -= step_interval
```

**Výhody:**
- Přesné časování pro více motorů současně
- Žádné blokující čekání
- Automatické zaokrouhlování chyb
- Konstantní ISR doba běhu

### Optimalizace pro 8-bit AVR

#### 16-bit tick aritmetika
Místo 32-bit microsekund používáme 16-bit ticky:

```cpp
// ❌ POMALÉ (32-bit):
uint32_t step_interval_us;
uint32_t accumulator_us;

// ✅ RYCHLÉ (16-bit):
uint16_t step_interval_ticks;
uint16_t accumulator_ticks;
```

**Performance gain**: 3-5× rychlejší ISR
- 32-bit operace: 10-15 CPU cyklů
- 16-bit operace: 2-4 CPU cyklů
- Polovina paměti (20 bytes vs 40 bytes)

#### Rozsah
```
Minimum: 3 ticks = 48 μs → 20.8 kHz max (bezpečnostní limit)
Maximum: 65535 ticks = 1.05 s → 0.95 Hz min
Prakticky: 1-5000 steps/s
```

### Odstranění dělení z ISR

**Problém**: Dělení je extrémně pomalé na AVR (≈100+ cyklů)

**Řešení**: Předvýpočet mimo ISR
```cpp
// MIMO ISR (v setAcceleration, setMicrostepping):
void updateAccelerationRate() {
    uint32_t accel = acceleration_;
    uint32_t total_steps = (uint32_t)steps_per_rev_ * microsteps_;
    uint16_t rpm_change = (accel * 672) / (total_steps * 1000);
    rpm_changes[motor_index_] = rpm_change;  // Uložit
}

// V ISR (BEZ dělení):
uint16_t rpm_change = rpm_changes[i];  // Jen načtení!
current += rpm_change;
```

## API Změny

### Původní vs. Nové API

#### ❌ Staré API (složité, pomalé)
```cpp
void setTargetSpeed(int16_t rpm);
void setMicrostepping(uint8_t divisor);
void setStepsPerRevolution(uint16_t steps);
// Interní výpočet: steps/s = (rpm × steps_per_rev × microsteps) / 60
```

#### ✅ Nové API (jednoduché, rychlé)
```cpp
void setTargetSpeed(int16_t steps_per_sec);  // Přímo steps/s
void setImmediateSpeed(int16_t steps_per_sec);
// Žádné přepočty RPM, žádné microstepping parametry
```

**Důvody změny:**
- Eliminace zbytečných výpočtů
- Uživatel má přímou kontrolu nad frekvencí kroků
- Jednodušší implementace
- Rychlejší ISR

### Softwarová omezení
```cpp
void setTargetSpeed(int16_t steps_per_sec) {
    if (steps_per_sec > 5000) steps_per_sec = 5000;
    if (steps_per_sec < -5000) steps_per_sec = -5000;
    // Automatické oříznutí na bezpečný rozsah
}
```

## Datové struktury

### Statické pole (sdílené s ISR)
```cpp
static constexpr uint8_t MAX_STEPPERS = 5;

volatile uint8_t stepper_count_;
volatile uint8_t step_pins_[MAX_STEPPERS];
volatile uint8_t dir_pins_[MAX_STEPPERS];
volatile int16_t current_speeds_[MAX_STEPPERS];  // Aktuální rychlost (se znaménkem!)
volatile int16_t target_speeds_[MAX_STEPPERS];   // Cílová rychlost
```

### Namespace proměnné (interní ISR)
```cpp
namespace {
    volatile uint16_t step_accumulators[5];    // Akumulátor v tickách (16-bit!)
    volatile uint16_t step_intervals[5];       // Interval mezi kroky v tickách
    volatile uint16_t acceleration_rates[5];   // steps/s²
    volatile uint16_t rpm_changes[5];          // Předpočítaná změna rychlosti
    volatile uint8_t accel_counter[5];         // Počítadlo pro akceleraci
    volatile bool step_pin_high[5];            // Sledování HIGH pinu
}
```

## ISR - OnTimer1StepperISR()

### Struktura ISR (každých 112 μs)

```cpp
void OnTimer1StepperISR() {
    // FÁZE 1: Clear pulzy z minulého ISR (112 μs pulse width)
    for (uint8_t i = 0; i < stepper_count_; i++) {
        if (step_pin_high[i]) {
            digitalWrite(step_pins_[i], LOW);
            step_pin_high[i] = false;
        }
    }
    
    // FÁZE 2: Zpracování každého motoru
    for (uint8_t i = 0; i < stepper_count_; i++) {
        // 2A: Akcelerace (každých 100 ISR = 11.2 ms)
        accel_counter[i]++;
        if (accel_counter[i] >= 100) {
            accel_counter[i] = 0;
            
            if (current != target) {
                uint16_t rpm_change = rpm_changes[i];  // Předpočítáno!
                current += (current < target) ? rpm_change : -rpm_change;
                updateStepInterval();  // Přepočítat interval
            }
        }
        
        // 2B: Generování kroků (accumulator approach)
        if (current_speed != 0) {  // ✅ Pouze pokud motor běží
            step_accumulators[i] += 7;  // ISR_PERIOD_TICKS
            
            if (step_accumulators[i] >= step_intervals[i]) {
                digitalWrite(step_pins_[i], HIGH);
                step_pin_high[i] = true;
                step_accumulators[i] -= step_intervals[i];
            }
        }
    }
}
```

### Timing kroků
```
ISR N:   Set pin HIGH, mark in step_pin_high[]
ISR N+1: Clear pin LOW (112 μs pulse width)
```

**Proč split pulse?**
- Čistý kód (není potřeba delay v ISR)
- Konzistentní pulse width (112 μs)
- ISR zůstává rychlý

### Akcelerace
- **Frekvence**: Každých 100 ISR = 11.2 ms
- **Výpočet**: Předpočítaný `rpm_changes[i]`
- **Bez dělení**: Jen sčítání/odčítání!

## Metody

### updateStepInterval()
Převod rychlosti (steps/s) na interval (ticks):

```cpp
void updateStepInterval() {
    int16_t speed = current_speeds_[motor_index_];
    
    if (speed == 0) {
        step_intervals[motor_index_] = 62500;  // 1 Hz
        return;
    }
    
    uint16_t abs_speed = (speed < 0) ? -speed : speed;
    
    // interval_ticks = 62500 / steps_per_sec
    uint16_t interval_ticks = TICKS_PER_SECOND / abs_speed;
    if (interval_ticks < 3) interval_ticks = 3;  // Safety
    
    step_intervals[motor_index_] = interval_ticks;
}
```

**Příklad:**
```
100 steps/s → interval = 62500 / 100 = 625 ticks = 10 ms
1000 steps/s → interval = 62500 / 1000 = 62.5 ticks ≈ 1 ms
5000 steps/s → interval = 62500 / 5000 = 12.5 ticks ≈ 200 μs
```

### updateDirection()
```cpp
void updateDirection() {
    if (target_speeds_[motor_index_] >= 0) {
        digitalWrite(dir_pin_, HIGH);  // Clockwise
    } else {
        digitalWrite(dir_pin_, LOW);   // Counter-clockwise
    }
}
```

## Použití

### Základní setup
```cpp
#include "motor_control/stepper_continuous.h"

StepperContinuous stepper;
stepper.init(STEP_PIN, DIR_PIN);
stepper.setAcceleration(500);  // steps/s²
```

### Plynulý rozjezd/zastavení
```cpp
stepper.setTargetSpeed(1000);   // Akceleruje na 1000 steps/s
delay(5000);
stepper.setTargetSpeed(0);      // Decelerate to stop
```

### Okamžitá změna rychlosti
```cpp
stepper.setImmediateSpeed(500);  // Skočí na 500 steps/s BEZ rampy
// ⚠️ Varování: Může způsobit ztrátu kroků!
```

### Směr
```cpp
stepper.setTargetSpeed(800);    // +800 = Clockwise
stepper.setTargetSpeed(-800);   // -800 = Counter-clockwise
```

### Změna akcelerace
```cpp
stepper.setAcceleration(1000);  // Rychlejší rozjezd/brzdění
stepper.setAcceleration(100);   // Pomalejší, hladší
```

## Typické hodnoty

### Akcelerace
- **100-300 steps/s²**: Velmi hladký, pomalý rozjezd
- **500 steps/s²**: Výchozí, dobrý kompromis
- **1000+ steps/s²**: Rychlý rozjezd, může způsobit vibrace

### Rychlost (pro NEMA 17, 200 steps/rev)
- **200-800 steps/s**: Typické použití (60-240 RPM)
- **1000-2000 steps/s**: Vysoká rychlost (300-600 RPM)
- **5000 steps/s max**: Hardware limit (1500 RPM)

## Memory & Performance

### Paměť
```
Per motor overhead:
  Static arrays: 2 bytes (pins) + 4 bytes (speeds) = 6 bytes
  Namespace: 8 bytes (accumulators, intervals, counters)
  Total: 14 bytes per motor × 5 = 70 bytes

Instance size: ~12 bytes (members)
```

### ISR Performance
```
Empty loop: ~10 CPU cycles
Per motor processing: ~50-80 cycles
  - Acceleration check: 20 cycles
  - Accumulator: 10 cycles  
  - Pin writes: 20-40 cycles

Total ISR @ 5 motors: ~400-450 cycles @ 16 MHz = 25-28 μs
ISR period: 112 μs → CPU load ~22-25%
```

## Omezení & Poznámky

### Hardware limitace
- **Max 5 motorů**: Statické pole `MAX_STEPPERS = 5`
- **Shared timer**: Všechny motory sdílí Timer1
- **ISR overhead**: ~25% CPU při 5 motorech

### Software limitace
- **Rychlost**: -5000 až +5000 steps/s (automaticky oříznuté)
- **Akcelerace**: uint16_t (0-65535 steps/s²)

### Kdy použít setImmediateSpeed vs setTargetSpeed
**setTargetSpeed** (preferováno):
- Normální provoz
- Plynulý start/stop
- Změny rychlosti za běhu

**setImmediateSpeed** (opatrně):
- Start z nulové rychlosti
- Emergency stop
- Když víte co děláte

###Condici pro zastavení pulzů
```cpp
if (current_speed != 0) {
    // Generuj kroky
}
```
Když `current_speed == 0`, motor negeneruje žádné pulzy → žádná spotřeba, žádný pohyb.

## Debug & Diagnostics

### Ověření funkčnosti
```cpp
stepper.setTargetSpeed(1000);
delay(2000);  // Počkej na dosažení cílové rychlosti

int16_t current = stepper.getCurrentSpeed();
int16_t target = stepper.getTargetSpeed();

if (current == target) {
    // Motor dosáhl cílové rychlosti
}

bool moving = stepper.isMoving();  // true pokud speed != 0
```

### Simulátor
V simulátoru se měří frekvence pomocí fronty timestampů:
- Při každé náběžné hraně: přidat timestamp
- Každý tick: odstranit timestampy starší než 1s
- Výstup: velikost fronty = steps/s

## Best Practices

1. **Vždy používej akceleraci** (kromě startu z nuly)
2. **Testuj reálný hardware** - simulator není 100% přesný
3. **Nastavuj rozumnou akceleraci** - příliš vysoká způsobí ztrátu kroků
4. **Nepřekračuj 5000 steps/s** - hardware limit
5. **Pro pozicování** - použij budoucí `StepperPositioning` třídu (trapezoidal profile)
6. **Směr nastav před rozjezdem** - `setTargetSpeed` automaticky updatuje

## Comparison s DC Motor Control

| Feature | Stepper | DC Motor |
|---------|---------|----------|
| Timer | Timer1 (16-bit) | Timer0/2 (8-bit) |
| Prescaler | 256 | 64 |
| ISR freq | 8928 Hz | Variable |
| Control | Steps/s | PWM duty cycle |
| Direction | DIR pin | Polarity/H-bridge |
| Feedback | Open-loop | Optional encoder |
| Precision | ±1 step | Analog |
| Max motors | 5 | 8 |

## Závěr

Tato implementace poskytuje:
- ✅ Efektivní řízení až 5 stepper motorů
- ✅ Optimalizované pro 8-bit AVR
- ✅ Přesné časování s accumulator approach
- ✅ Plynulá akcelerace/decelerace
- ✅ Minimální CPU overhead (~25%)
- ✅ Jednoduché API (steps/s)

Pro positioning režim (absolutní poloha, trapezoidal profile) implementuj samostatnou třídu `StepperPositioning`.
