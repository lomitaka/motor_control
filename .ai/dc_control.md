# DC Motor Control - Technická Dokumentace

## Přehled
Implementace řízení DC motorů pro AVR ATmega328p pomocí software PWM s podporou až 8 motorů současně (5 DC + 3 servo).

## Soubory
- **Header**: `include/motor_control/dc_control.h`
- **Implementace**: `src/dc_control.cpp`
- **Timer control**: `src/internals/timer_control.h`, `src/timer_control.cpp`

## Hardware - Timer1 Konfigurace

### Základní parametry
- **CPU frekvence**: 16 MHz
- **Timer**: Timer1 (16-bit)
- **Režim**: CTC with OCR1A as TOP (WGM12)
- **Prescaler**: 1 (no prescaler - full speed)
- **OCR1A**: 63999 (defines PWM period)
- **OCR1B**: Dynamický (multiplexovaný pro motory)

### Výpočet timingu
```
Timer frequency = 16 MHz / 1 = 16 MHz
Tick period = 62.5 ns
OCR1A = 63999 → Period = 64000 × 62.5 ns = 4 ms
PWM frequency = 1 / 4 ms = 250 Hz
```

### Registry setup
```cpp
TCNT1 = 0;                          // Reset counter
OCR1A = 63999;                       // TOP value (4 ms period)
OCR1B = 65535;                       // Initial (won't trigger before overflow)
TIMSK1 = (1<<OCIE1A) | (1<<OCIE1B) | (1<<TOIE1);  // Enable interrupts
TCCR1A = 0;                          // Normal port operation
TCCR1B = (1<<WGM12) | (1<<CS10);    // CTC mode, no prescaler
```

## Architektura - Software PWM Multiplexing

### Hlavní koncept
Místo hardware PWM pinů používáme **time-multiplexed software PWM**:
- Jeden timer řídí všechny motory
- Compare Match B (OCR1B) se dynamicky mění
- Každý motor má svůj duty cycle (0-1000)

### PWM Princip
```
4 ms PWM period:
├── Overflow (t=0): All pins HIGH, sort motors by PWM value
├── Compare Match B (motor 0): Turn off first motor
├── Compare Match B (motor 1): Turn off second motor
├── Compare Match B (motor 2): Turn off third motor
├── Compare Match B (motor 3): Turn off fourth motor
├── Compare Match B (motor 4): Turn off fifth motor
└── Overflow (t=4ms): Repeat cycle
```

**Příklad:**
```
Motor A: PWM=300 (30%) → HIGH 0-1.2ms, LOW 1.2-4ms
Motor B: PWM=700 (70%) → HIGH 0-2.8ms, LOW 2.8-4ms
Motor C: PWM=500 (50%) → HIGH 0-2ms, LOW 2-4ms

Timeline:
0ms: All HIGH
1.2ms: Motor A → LOW
2ms: Motor C → LOW
2.8ms: Motor B → LOW
4ms: Overflow, repeat
```

### Buffer systém
```cpp
volatile uint16_t dc_motors_[5];        // Aktuální hodnoty (zapisuje uživatel)
volatile uint16_t dc_motors_buffer_[5]; // Buffer (používá ISR)
```

**Důvod:**
- Zabraňuje race conditions
- ISR pracuje s konzistentními daty po celý PWM cyklus
- Buffer se aktualizuje jen při overflow (každé 4 ms)

## ISR Struktura

### 1. Overflow ISR (každé 4 ms)
```cpp
ISR(TIMER1_OVF_vect) {
    // 1. Copy user values to buffer
    for (uint8_t i = 0; i < dc_motor_count_; i++) {
        dc_motors_buffer_[i] = dc_motors_[i];
    }
    
    // 2. Sort motors by PWM value (create off_order array)
    sortMotorsByPWM();  // dc_motors_off_order[0..4]
    
    // 3. Set all pins HIGH
    for (uint8_t i = 0; i < dc_motor_count_; i++) {
        setPinHigh(dc_port_pwm_pin_[i]);
    }
    
    // 4. Schedule first Compare Match
    OCR1B = calculateNextCompare(0);
    dc_current_index_ = 0;
}
```

### 2. Compare Match B ISR (dynamický timing)
```cpp
ISR(TIMER1_COMPB_vect) {
    // 1. Turn off current motor
    uint8_t motor_idx = dc_motors_off_order[dc_current_index_];
    setPinLow(dc_port_pwm_pin_[motor_idx]);
    
    // 2. Move to next motor
    dc_current_index_++;
    
    // 3. Schedule next Compare Match (if more motors)
    if (dc_current_index_ < dc_motor_count_) {
        OCR1B = calculateNextCompare(dc_current_index_);
    }
}
```

### Guard Time
```cpp
constexpr uint16_t TIMER_GUARD_TICKS = 100;

uint16_t calculateNextCompare(uint8_t index) {
    uint16_t current = TCNT1;
    uint16_t target = dc_motors_buffer_[index] * 64;  // Scale 0-1000 → 0-64000
    
    // Ensure target is at least TIMER_GUARD_TICKS ahead
    if (target < current + TIMER_GUARD_TICKS) {
        target = current + TIMER_GUARD_TICKS;
    }
    
    return target;
}
```

**Proč?**
- Zabraňuje missingu compare match
- ISR trvá ~50-100 CPU cyklů
- 100 ticks = 6.25 μs margin

## Třídění motorů

### Insertion Sort
```cpp
void sortMotorsByPWM() {
    // Create index array
    for (uint8_t i = 0; i < dc_motor_count_; i++) {
        dc_motors_off_order[i] = i;
    }
    
    // Insertion sort by PWM value (ascending)
    for (uint8_t i = 1; i < dc_motor_count_; i++) {
        uint8_t key = dc_motors_off_order[i];
        uint16_t key_pwm = dc_motors_buffer_[key];
        int8_t j = i - 1;
        
        while (j >= 0 && dc_motors_buffer_[dc_motors_off_order[j]] > key_pwm) {
            dc_motors_off_order[j + 1] = dc_motors_off_order[j];
            j--;
        }
        dc_motors_off_order[j + 1] = key;
    }
}
```

**Proč Insertion Sort?**
- Malý dataset (max 5 prvků)
- Často už částečně setříděné
- Minimální overhead (~50-80 cyklů)
- Stable sort (zachovává pořadí při rovnosti)

## Datové struktury

### Static arrays (sdílené s ISR)
```cpp
static constexpr uint8_t MAX_MOTOR_CNT = 5;

volatile uint8_t dc_motor_count_;           // Počet aktivních motorů
volatile uint8_t dc_motor_count_buffer_;    // Buffer count
volatile uint8_t dc_current_index_;         // Aktuální motor v ISR

volatile uint8_t dc_motors_off_order[5];    // Setříděné indexy
volatile uint16_t dc_motors_[5];            // PWM hodnoty (0-1000)
volatile uint16_t dc_motors_buffer_[5];     // ISR buffer
volatile uint8_t dc_port_pwm_pin_[5];       // PWM pin kódy
volatile uint8_t dc_port_dir_pin_[5];       // Direction pin kódy
```

### Pin encoding
```cpp
// Pin kód: [4 bits port | 4 bits pin]
// Port: 2=PORTB, 3=PORTC, 4=PORTD
// Pin: 0-7

Example: Pin 9 (Arduino) = PORTB1
  → port_pin_code = 0x21 (port=2, pin=1)
```

## API

### Konstruktor a inicializace
```cpp
DCControl motor(pwm_pin, direction_pin);
// nebo
DCControl motor;
motor.init(pwm_pin, direction_pin);
```

### Řízení rychlosti
```cpp
void setSpeed(int16_t speed);  // -1000 až +1000
```

**Rozsahy:**
- `-1000`: Plná rychlost zpět
- `0`: Stop
- `+1000`: Plná rychlost vpřed

**Interní:**
```cpp
void DCControl::setSpeed(int16_t speed) {
    // Clamp to range
    if (speed > 1000) speed = 1000;
    if (speed < -1000) speed = -1000;
    
    // Set direction
    if (speed >= 0) {
        digitalWrite(port_dir_index_, HIGH);
        dc_motors_[motor_index_] = speed;
    } else {
        digitalWrite(port_dir_index_, LOW);
        dc_motors_[motor_index_] = -speed;  // Absolute value
    }
}
```

### Status metody
```cpp
int16_t getSpeed();           // Aktuální rychlost
int16_t getTargetSpeed();     // Cílová rychlost (s rampou)
uint8_t getLastError();       // Error kód
```

## Použití

### Základní setup
```cpp
#include "motor_control/dc_control.h"

DCControl motor(9, 10);  // PWM pin 9, direction pin 10
motor.setSpeed(500);     // 50% speed forward
```

### S rampou (smooth control)
```cpp
#include "motor_control/dc_control.h"
#include "motor_control/ramp.h"

DCControl motor(9, 10);
Ramp ramp;

motor.setRamp(&ramp);
ramp.setSpeed(800, 500);  // Target 800, accel 500
ramp.start();

while (ramp.isActive()) {
    motor.updateRamp();
    delay(10);
}
```

### Více motorů
```cpp
DCControl motor1(9, 10);
DCControl motor2(5, 6);
DCControl motor3(3, 4);

motor1.setSpeed(700);   // Motor 1: 70% forward
motor2.setSpeed(-500);  // Motor 2: 50% backward
motor3.setSpeed(0);     // Motor 3: stop
```

### Směr a rychlost
```cpp
motor.setSpeed(600);    // 60% forward (direction pin HIGH)
delay(1000);
motor.setSpeed(-600);   // 60% backward (direction pin LOW)
delay(1000);
motor.setSpeed(0);      // Stop
```

## Timing & Performance

### PWM Charakteristiky
```
Frequency: 250 Hz (4 ms period)
Resolution: 1000 levels (0.1% steps)
Actual resolution: 64000 timer ticks → ~16000:1 ratio
Effective: 10-bit PWM (1024 levels, but using 1000)
```

### ISR Overhead
```
Overflow ISR:
  - Copy buffer: ~20 cycles
  - Sort (5 motors): ~80 cycles
  - Set pins HIGH: ~100 cycles
  Total: ~200 cycles = 12.5 μs

Compare Match ISR:
  - Set pin LOW: ~40 cycles
  - Update OCR1B: ~20 cycles
  Total: ~60 cycles = 3.75 μs

Per 4ms cycle (5 motors):
  - 1× Overflow: 12.5 μs
  - 5× Compare Match: 18.75 μs
  Total: 31.25 μs / 4000 μs = 0.78% CPU
```

### Memory Usage
```
Per motor:
  - PWM value: 2 bytes
  - Buffer: 2 bytes
  - Pins: 2 bytes
  Total: 6 bytes per motor × 5 = 30 bytes

Instance size: ~8 bytes (members)
Static overhead: ~40 bytes (arrays, counters)
```

## Omezení & Poznámky

### Hardware limitace
- **Max 5 DC motorů** (shared with servo - total 8 slots)
- **Shared Timer1**: DC motory a serva sdílí stejný timer
- **No true hardware PWM**: Software emulace

### PWM limitace
- **Frequency**: Fixní 250 Hz
- **Resolution**: 0-1000 (0.1% steps)
- **Minimum pulse**: Guard time ~6 μs

### Pin requirements
- **2 piny per motor**: PWM + direction
- **Total**: až 10 pinů (5 motorů × 2)

## Comparison s Stepper Control

| Feature | DC Motor | Stepper |
|---------|----------|---------|
| Timer | Timer1 (16-bit) | Timer1 (16-bit) |
| Prescaler | 1 (16 MHz) | 256 (62.5 kHz) |
| Period | 4 ms | 112 μs |
| Control | PWM duty cycle | Step frequency |
| Max devices | 5 DC + 3 servo | 5 steppers |
| CPU load | ~0.8% | ~25% |
| Precision | Analog (0.1%) | Digital (±1 step) |
| Direction | DIR pin | DIR pin |
| Feedback | Optional encoder | Open-loop |

## Direction Control

### H-Bridge připojení
```
DC Motor Control → H-Bridge Driver (L298N, L293D, etc.)

PWM Pin → Enable (Speed control)
DIR Pin → Input 1/2 (Direction control)

DIR=HIGH: Forward
DIR=LOW: Backward
```

### Single Direction Mode
```cpp
// Pokud motor jede jen jedním směrem:
digitalWrite(direction_pin, HIGH);  // Always forward
motor.setSpeed(abs(speed));         // Ignore sign
```

## Error Handling

### Error kódy
```cpp
enum ErrorCodes {
    ERROR_NO_FREE_MOTOR = 1,    // Všechny sloty obsazené
    ERROR_INVALID_PIN = 2,      // Neplatný pin
    ERROR_TIMER_NOT_INIT = 3    // Timer nebyl inicializován
};
```

### Diagnostika
```cpp
if (motor.getLastError() != 0) {
    // Handle error
    uint8_t error = motor.getLastError();
    if (error == DCControl::ERROR_NO_FREE_MOTOR) {
        // Too many motors
    }
}
```

## Best Practices

1. **Inicializuj motor před použitím**
   ```cpp
   motor.init(pwm_pin, dir_pin);
   ```

2. **Používej rampy pro plynulé změny**
   ```cpp
   motor.setRamp(&ramp);
   motor.updateRamp();
   ```

3. **Nepřekračuj 5 motorů** - hardware limit

4. **Testuj na reálném hardware** - PWM timing kritický

5. **Vnější obvody**:
   - H-bridge pro směr + brždění
   - Flyback diody pro zpětnou EMF
   - Kondenzátory pro filtraci

6. **Kalibrace**:
   - Najdi minimální PWM pro rozběh (~100-200)
   - Nastavuj rozsah podle zátěže

7. **Power management**:
   ```cpp
   motor.setSpeed(0);  // Stop when not needed
   ```

## Debugging

### Ověření PWM výstupu
```cpp
motor.setSpeed(500);  // 50% duty cycle
// Measure with oscilloscope:
// - Frequency should be 250 Hz (4 ms period)
// - Duty cycle should be 50% (2 ms HIGH, 2 ms LOW)
```

### Diagnostika timing
```cpp
// V simulátoru:
simulator.configurePin(pwm_pin, MotorType::DC_MOTOR);
// Monitor PWM frequency and duty cycle
double load = simulator.getCurrentLoad(pwm_pin);  // Should match setSpeed()
```

## Integration s ostatními komponenty

### Servo + DC motors (shared timer)
```cpp
// DC motory a serva sdílí Timer1
// Max 5 DC + 3 servo = 8 total

DCControl dc1(9, 10);
DCControl dc2(5, 6);
ServoControl servo1;  // Uses same timer infrastructure

// Timer je sdílený, ale ISR handler jsou separátní
```

### Compatibility
- ✅ Kompatibilní se ServoControl (shared Timer1)
- ❌ Nekompatibilní se StepperControl (conflicts on Timer1)
- ⚠️ Pro mixed použití: Use different timers nebo software scheduling

## Závěr

DC Motor Control poskytuje:
- ✅ Software PWM pro až 5 DC motorů
- ✅ 250 Hz @ 0.1% resolution
- ✅ Minimální CPU overhead (~0.8%)
- ✅ Smooth rampa support
- ✅ Bidirectional control (forward/backward)
- ✅ Simple API (speed -1000 to +1000)

Pro high-performance aplikace s více motory zvažte:
- External PWM driver (PCA9685)
- Hardware PWM pins (Timer0/2 - omezený počet)
- Separate MCU pro motor control
