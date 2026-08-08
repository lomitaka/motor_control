# TODO - Zbývající úkoly

## Datum vytvoření: 2026-08-08



## 📋 OTEVŘENÉ ÚKOLY

### 1. Stepper Positioning - Testování

**TODO:**
- [ ] Vytvořit test pro positioning control
- [ ] Ověřit správnost braking distance výpočtu
- [ ] Otestovat na reálném hardware

---

### 2. Konflikty mezi timer systémy

**Problém:** Všechny tři systémy používají Timer1:
1. DC Control + Servo Control: Timer1, no prescaler, CTC @ 64000
2. Stepper Continuous: Timer1, prescaler 256, CTC @ 7
3. Stepper Positioning: Timer1, no prescaler, dynamic OCR1A

**Důsledek:** Nelze kombinovat různé režimy současně.

**Doporučení:** Přidat do dokumentace jasné varování:
```
⚠️ TIMER CONFLICTS:
- DC/Servo control CANNOT run with Stepper Continuous
- DC/Servo control CANNOT run with Stepper Positioning
- Stepper Continuous CANNOT run with Stepper Positioning
- Choose ONE control system per application
```

---

### 3. Stepper Positioning - Optimalizace ISR

**Soubor:** `src/stepp_control.cpp`, funkce `OnTimer1StepperPositioningOCRA()`

**Možná optimalizace:** 
- ISR aktuálně počítá braking distance a akceleraci pro každý motor při každém eventu
- Zvážit přesun akcelerace do samostatného časovače (např. každých N kroků)
- Možné ušetření CPU času v ISR

**Status:** 🔍 K posouzení po prvním testování

---

## Poznámky k implementaci Stepper Positioning:

### Klíčové rozdíly oproti Stepper Continuous:
- **Timer:** No prescaler (16 MHz) vs prescaler 256 (62.5 kHz)
- **Scheduling:** Dynamic OCR1A (nearest event) vs fixed ISR period
- **Position tracking:** Tracks remaining_ticks vs no position tracking
- **Braking:** Automatic braking distance calculation vs manual control
- **API:** Step-based (setTargetTicks) vs speed-based (setTargetSpeed)

### Design rozhodnutí:
- Použití binárního masky pro sledování HIGH pinů (efektivnější než pole bool)
- OCRB fixed delay (STEP_PULSE_TICKS) po OCRA pro jednoduchost
- Braking distance se počítá z v² / (2a) - standardní kinematika
- Akcelerace per-step místo per-time (jednodušší, ale méně přesné)

### Možná vylepšení v budoucnu:
- Time-based acceleration místo step-based (přesnější)
- Předvýpočet celé trajektorie (trapezoidal profile)
- S-křivka místo lineární ramp (plynulejší pohyb)
