# Motor Control Library - AI Context Documentation

Tato složka obsahuje technickou dokumentaci motor control knihovny pro použití jako kontext v AI konverzacích.

## Obsah

### 📄 [stepper_control.md](stepper_control.md)
**Stepper Motor Control - Kompletní dokumentace**

Popisuje implementaci řízení krokových motorů s následujícími tématy:
- ✅ Timer1 konfigurace (prescaler 256, CTC mode, 112 μs ISR)
- ✅ Accumulator approach pro přesné generování kroků
- ✅ 16-bit tick aritmetika (optimalizace pro 8-bit AVR)
- ✅ Odstranění dělení z ISR (předvýpočet akcelerace)
- ✅ API změny (RPM → steps/s)
- ✅ ISR struktura a timing
- ✅ Memory & performance analýza
- ✅ Best practices & debugging

**Kdy použít:** Pro otázky o stepper motor řízení, timing optimalizacích, ISR implementaci, nebo obecně o přesném step generování.

### 📄 [dc_control.md](dc_control.md)
**DC Motor Control - Kompletní dokumentace**

Popisuje implementaci software PWM pro DC motory s tématy:
- ✅ Timer1 konfigurace (no prescaler, 250 Hz PWM)
- ✅ Software PWM multiplexing (až 5 motorů)
- ✅ Buffer systém a race condition prevention
- ✅ Insertion sort pro motor ordering
- ✅ Rampa support (smooth acceleration)
- ✅ H-bridge integration
- ✅ Bidirectional control
- ✅ Timing & performance analýza

**Kdy použít:** Pro otázky o DC motor PWM řízení, software PWM multiplexingu, nebo bidirectional motor control.

---

## Rychlý přehled - Stepper vs DC

| Vlastnost | Stepper Control | DC Control |
|-----------|----------------|------------|
| **Timer** | Timer1 (16-bit) | Timer1 (16-bit) |
| **Prescaler** | 256 | 1 (full speed) |
| **ISR Period** | 112 μs | 4 ms (overflow) |
| **Control Type** | Step frequency (steps/s) | PWM duty cycle (0-1000) |
| **Max Devices** | 5 steppers | 5 DC motors |
| **CPU Load** | ~25% @ 5 motors | ~0.8% @ 5 motors |
| **Precision** | ±1 step (digital) | 0.1% (analog) |
| **Direction** | DIR pin (digital) | DIR pin + H-bridge |
| **Acceleration** | Built-in rampa | External Ramp object |
| **Feedback** | Open-loop | Optional encoder |

---

## Klíčové koncepty

### Stepper Control
1. **Accumulator approach**: Přesný step timing bez delay
2. **16-bit optimization**: 3-5× rychlejší než 32-bit
3. **No division in ISR**: Předvýpočet akcelerace
4. **Split pulse**: Set HIGH → wait 112 μs → clear LOW

### DC Control
1. **Software PWM multiplexing**: Jeden timer, více motorů
2. **Buffer system**: Race condition prevention
3. **Dynamic OCR1B**: Řazení motorů podle duty cycle
4. **Guard time**: Prevence missed compare match

---

## Typické use-cases

### Stepper Motor
```cpp
#include "motor_control/stepper_continuous.h"

StepperContinuous stepper;
stepper.init(STEP_PIN, DIR_PIN);
stepper.setAcceleration(500);       // steps/s²
stepper.setTargetSpeed(1000);        // 1000 steps/s
// Motor plynule akceleruje na cílovou rychlost
```

### DC Motor
```cpp
#include "motor_control/dc_control.h"

DCControl motor(PWM_PIN, DIR_PIN);
motor.setSpeed(700);                 // 70% forward
delay(1000);
motor.setSpeed(-500);                // 50% backward
```

---

## Architektura projektu

```
include/motor_control/
├── stepper_continuous.h      # Stepper API
├── stepper_positioning.h     # Positioning mode (TODO)
├── dc_control.h              # DC motor API
├── servo_control.h           # Servo API (shared Timer1)
└── ramp.h                    # Acceleration helper

src/
├── step_control.cpp          # Stepper implementation
├── step_timer_control.cpp    # Stepper Timer1 setup
├── dc_control.cpp            # DC motor implementation
├── timer_control.cpp         # Shared Timer1 control
└── internals/
    ├── step_timer_control.h  # Stepper timer header
    └── timer_control.h       # Shared timer header
```

---

## Memory Budget

### Stepper Control
```
Static arrays: 70 bytes (5 motors × 14 bytes)
Per instance: 12 bytes
ISR overhead: ~25% CPU @ 5 motors
```

### DC Control
```
Static arrays: 30 bytes (5 motors × 6 bytes)
Per instance: 8 bytes
ISR overhead: ~0.8% CPU @ 5 motors
```

---

## Best Practices Summary

### Stepper Motors
1. ✅ Vždy používej `setTargetSpeed()` s akcelerací
2. ✅ Respektuj limit 5000 steps/s
3. ✅ Testuj na reálném hardware (timing kritický)
4. ⚠️ `setImmediateSpeed()` pouze pro start z nuly
5. ⚠️ Vysoká akcelerace může způsobit ztrátu kroků

### DC Motors
1. ✅ Inicializuj motor před použitím
2. ✅ Používej rampy pro plynulé změny
3. ✅ External H-bridge + flyback diodes
4. ⚠️ Nepřekračuj 5 motorů (hardware limit)
5. ⚠️ PWM frequency fixní (250 Hz)

---

## Compatibility Matrix

| Komponenta | Timer1 | Compatible with Stepper | Compatible with DC |
|------------|--------|------------------------|-------------------|
| **StepperContinuous** | ✅ Exclusive | N/A | ❌ Conflict |
| **DCControl** | ✅ Shared | ❌ Conflict | ✅ Yes |
| **ServoControl** | ✅ Shared | ❌ Conflict | ✅ Yes |

**Poznámka:** Stepper a DC/Servo **nelze kombinovat** (oba používají Timer1).

---

## Debugging Tips

### Stepper
```cpp
stepper.getCurrentSpeed();   // Aktuální rychlost
stepper.getTargetSpeed();    // Cílová rychlost
stepper.isMoving();          // true pokud speed != 0
```

### DC Motor
```cpp
motor.getSpeed();            // Aktuální PWM hodnota
motor.getLastError();        // Error kód
// Oscilloscope: 250 Hz, correct duty cycle
```

### Simulator
```cpp
simulator.configurePin(pin, MotorType::STEPPER);
double freq = simulator.getCurrentLoad(pin);  // steps/s

simulator.configurePin(pin, MotorType::DC_MOTOR);
double duty = simulator.getCurrentLoad(pin);  // 0-100%
```

---

## Common Issues & Solutions

### Stepper
❌ **Motor se netočí**
- Check `current_speed != 0` condition
- Verify timer initialized
- Check pin connections

❌ **Ztráta kroků**
- Snižte akceleraci
- Snižte max rychlost
- Zvyšte napájecí napětí

### DC Motor
❌ **Žádný PWM výstup**
- Verify timer initialized
- Check pin configuration
- Verify motor slot available

❌ **Hrubé PWM**
- Normal @ 250 Hz
- Use external filter/capacitor

---

## Future Work

### Stepper Positioning Mode
```cpp
// TODO: Implementovat
StepperPositioning stepper;
stepper.moveTo(1000);           // Absolute position
stepper.moveBy(500);            // Relative move
stepper.setMaxSpeed(2000);
stepper.waitUntilDone();
```

### Hardware PWM Option
```cpp
// TODO: Využít Timer0/2 pro hardware PWM
// Omezené piny, ale vyšší frequency
```

---

## Technické detaily

### AVR Optimization
- **16-bit aritmetika**: 2-4 CPU cyklů
- **32-bit aritmetika**: 10-15 CPU cyklů
- **Dělení**: ~100+ CPU cyklů
- **ISR latency**: ~10-20 cyklů

### Timer Resources
- **Timer0**: System timing (millis/micros) - 8-bit
- **Timer1**: Motor control (DC/Servo/Stepper) - 16-bit
- **Timer2**: Available for other use - 8-bit

### Interrupt Priorities
```
Highest: Timer1 COMPA (stepper steps)
         Timer1 COMPB (DC motor PWM)
         Timer1 OVF (overflow)
Lowest:  User interrupts
```

---

## Contact & Contribution

Pro otázky, bug reporty, nebo contribution:
- Přečti si relevantní .md soubor
- Check progress.txt pro známé issues
- Testuj na simulátoru i reálném hardware

**Simulator:**
```bash
cd simulator/build
cmake .. && make
./stepper_test      # Test stepper motors
./motor_simulator   # Test DC/servo
```

---

## License & Attribution

Tato dokumentace je součástí motor control library projektu pro AVR ATmega328p.
Vytvořeno pro akademické účely (ZS2025/MICRO/zapoctak).

**Version:** 1.0  
**Last Updated:** 2026-08-08  
**Platform:** AVR ATmega328p @ 16 MHz
