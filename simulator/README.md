# AVR Timer Simulator

Tento simulátor umožňuje testovat ovladače motorů bez skutečného AVR hardwaru.

## Co simulátor dělá:

1. **Simuluje Timer1** - inkrementuje TCNT1, detekuje overflow a compare match
2. **Simuluje GPIO piny** - PORTB, PORTC, PORTD
3. **Volá ISR funkce** - když nastanou přerušení
4. **Loguje změny pinů** - do souboru s časovými značkami

## Jak použít:

### 1. Příprava kódu pro simulaci

Ve vašich zdrojových souborech přidejte podmíněnou kompilaci:

```cpp
#ifdef SIMULATION_MODE
    #include "avr_mock.h"
#else
    #include <avr/io.h>
    #include <avr/interrupt.h>
#endif
```

### 2. Build simulátoru

```bash
cd simulator
mkdir build
cd build
cmake ..
make
```

### 3. Spuštění simulace

```bash
./motor_simulator
```

### 4. Kontrola výsledků

Otevřete `motor_simulation.log` a prohlédněte si změny pinů v čase.

## Příklad výstupu logu:

```
AVR Timer1 Simulation Log
CPU Frequency: 16000000 Hz
======================================

Starting simulation for 1.000000 seconds
Total cycles: 16000000
Timestep: 62.500 us (1 cycles)
======================================

[      0.000 us] PORTB.1 = HIGH  (TCNT1=0)
[   1500.000 us] PORTB.1 = LOW   (TCNT1=24000)
[   4096.000 us] PORTB.2 = HIGH  (TCNT1=0)
[   5596.000 us] PORTB.2 = LOW   (TCNT1=24000)
...
```

## Parametry simulace:

- **duration_seconds**: Délka simulace v sekundách (např. 1.0, 5.0, 500.0)
- **timestep_us**: Časový krok v mikrosekundách (výchozí 62.5 µs = 1 CPU cyklus @ 16 MHz)

Pro rychlejší simulaci použijte větší timestep (např. 100 µs), ale s menší přesností.

## Výhody:

✅ Testování bez hardwaru  
✅ Rychlé iterace (500s simulace běží za pár sekund)  
✅ Přesné časování a logování  
✅ Žádné změny v původním kódu (pouze podmíněná kompilace)  
✅ Detekce chyb dříve než na hardwaru

## Omezení:

⚠️ Simuluje pouze Timer1 a základní GPIO  
⚠️ Nepodporuje všechny AVR periferie  
⚠️ Delay funkce se ignorují (nečekají skutečně)
