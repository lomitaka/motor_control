#ifndef AVR_SIMULATOR_H
#define AVR_SIMULATOR_H

#include <stdint.h>
#include <functional>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <map>
#include <queue>

// Typy motorů
enum class MotorType {
    NONE,       // Pin není konfigurován
    SERVO,      // Servo motor (1-2ms pulzy)
    DC_MOTOR,   // DC motor (PWM řízení)
    STEPPER     // Stepper motor (budoucí implementace)
};

// Simulované AVR registry
namespace AVRSim {
    // Timer1 registry
    extern volatile uint16_t TCNT1;
    extern volatile uint16_t OCR1A;
    extern volatile uint16_t OCR1B;
    extern volatile uint8_t TCCR1A;
    extern volatile uint8_t TCCR1B;
    extern volatile uint8_t TIMSK1;
    extern volatile uint8_t TIFR1;
    
    // Port registry (B, C, D)
    extern volatile uint8_t PORTB;
    extern volatile uint8_t PORTC;
    extern volatile uint8_t PORTD;
    extern volatile uint8_t DDRB;
    extern volatile uint8_t DDRC;
    extern volatile uint8_t DDRD;
    extern volatile uint8_t PINB;
    extern volatile uint8_t PINC;
    extern volatile uint8_t PIND;
    
    // Interrupt flag
    extern volatile uint8_t SREG;
    
    // Konstanty pro bity (deklarace)
    extern const uint8_t CS10;
    extern const uint8_t CS11;
    extern const uint8_t CS12;
    extern const uint8_t WGM12;
    extern const uint8_t OCIE1A;
    extern const uint8_t OCIE1B;
    extern const uint8_t TOIE1;
    extern const uint8_t TOV1;
    extern const uint8_t OCF1A;
    extern const uint8_t OCF1B;
}

// Arduino-like funkce pro simulaci
void digitalWrite(uint8_t pin, uint8_t value);

// Struktura pro sledování zatížení pinu
struct PinMonitor {
    MotorType motorType;
    uint64_t lastChangeTime_us;     // Čas poslední změny (us)
    bool currentState;               // Aktuální stav (HIGH/LOW)
    uint64_t lastRisingEdge_us;     // Čas posledního nábězné hrany
    uint64_t lastFallingEdge_us;    // Čas posledního sestupné hrany
    double lastPulseWidth_us;       // Délka posledního HIGH pulzu
    double lastPeriod_us;           // Délka poslední periody
    double currentLoad;             // Aktuální zatížení v procentech
    std::queue<uint64_t> stepTimestamps;  // Fronta timestampů step pulzů (pro měření frekvence)
    
    PinMonitor() 
        : motorType(MotorType::NONE)
        , lastChangeTime_us(0)
        , currentState(false)
        , lastRisingEdge_us(0)
        , lastFallingEdge_us(0)
        , lastPulseWidth_us(0)
        , lastPeriod_us(0)
        , currentLoad(0.0)
    {}
};

// Simulátor třídy
class AVRTimerSimulator {
public:
    AVRTimerSimulator(const std::string& logFile = "simulation.log");
    ~AVRTimerSimulator();
    
    // Registrace ISR callbacků
    void registerCompareMatchA_ISR(std::function<void()> callback);
    void registerOverflow_ISR(std::function<void()> callback);
    void registerCompareMatchB_ISR(std::function<void()> callback);
    void registerDebugLog(std::function<void(uint32_t)> callback);
    
    // Simulace časování
    void simulate(double duration_seconds, double timestep_us = 62.5);
    
    // Nastavení CPU frekvence (default 16 MHz)
    void setCPUFrequency(uint32_t freq_hz);
    
    // Konfigurace typu motoru na pinu (Arduino pin numbering)
    void configurePin(uint8_t arduinoPin, MotorType type);
    
    // Logování
    void logPinChange(char port, uint8_t pin, bool state);
    void logTimerState();
    void logValues();
    void logMotorLoads();  // Logování zatížení všech motorů
    
private:
    void tick();  // Jeden časový krok (1 CPU cyklus)
    void checkInterrupts();
    uint16_t getPrescaler();
    void updatePinMonitor(uint8_t arduinoPin, bool state);  // Aktualizace sledování pinu
    double calculateServoLoad(const PinMonitor& monitor);   // Výpočet zatížení serva
    double calculateDCLoad(const PinMonitor& monitor);      // Výpočet zatížení DC motoru
    double calculateStepLoad(PinMonitor& monitor);           // Výpočet frekvence step pulzů (steps/s)
    uint8_t arduinoPinToPort(uint8_t arduinoPin, uint8_t& portPin);  // Převod Arduino pin -> PORT
    
    std::ofstream logFile_;
    std::ofstream loadLogFile_;  // Samostatný log pro zatížení motorů
    uint64_t cycleCount_;
    uint32_t cpuFrequency_;
    uint16_t prescalerCounter_;
    
    // Předchozí stavy portů pro detekci změn
    uint8_t prevPORTB_;
    uint8_t prevPORTC_;
    uint8_t prevPORTD_;
    
    // Sledování pinů podle Arduino číslování (0-19)
    std::map<uint8_t, PinMonitor> pinMonitors_;
    
    // Čas poslední reportu zatížení
    uint64_t lastLoadReport_us_;
    double loadReportInterval_us_;  // Interval reportování (default 20ms)
    
    // ISR callbacky
    std::function<void()> compareMatchA_ISR_;
    std::function<void()> overflow_ISR_;
    std::function<void()> compareMatchB_ISR_;
    std::function<void(uint32_t)> debugLog_;
    
    bool interruptsEnabled_;
};

// Makro pro simulaci sei()
#define sei() AVRSim::SREG |= 0x80

// Makro pro simulaci cli()
#define cli() AVRSim::SREG &= ~0x80

#endif // AVR_SIMULATOR_H
