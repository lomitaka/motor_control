#include "avr_simulator.h"
#include <iomanip>
#include <chrono>

// Definice simulovaných registrů
namespace AVRSim {
    volatile uint16_t TCNT1 = 0;
    volatile uint16_t OCR1A = 0;
    volatile uint16_t OCR1B = 0;
    volatile uint8_t TCCR1A = 0;
    volatile uint8_t TCCR1B = 0;
    volatile uint8_t TIMSK1 = 0;
    volatile uint8_t TIFR1 = 0;
    
    volatile uint8_t PORTB = 0;
    volatile uint8_t PORTC = 0;
    volatile uint8_t PORTD = 0;
    volatile uint8_t DDRB = 0;
    volatile uint8_t DDRC = 0;
    volatile uint8_t DDRD = 0;
    volatile uint8_t PINB = 0;
    volatile uint8_t PINC = 0;
    volatile uint8_t PIND = 0;
    
    volatile uint8_t SREG = 0x80;  // Interrupts enabled by default
    
    // Konstanty pro bity (definice)
    const uint8_t CS10 = 0;
    const uint8_t CS11 = 1;
    const uint8_t CS12 = 2;
    const uint8_t OCIE1A = 1;
    const uint8_t OCIE1B = 2;
    const uint8_t TOIE1 = 0;
    const uint8_t TOV1 = 0;
    const uint8_t OCF1A = 1;
    const uint8_t OCF1B = 2;
}

AVRTimerSimulator::AVRTimerSimulator(const std::string& logFile)
    : cycleCount_(0)
    , cpuFrequency_(16000000)  // 16 MHz default
    , prescalerCounter_(0)
    , prevPORTB_(0)
    , prevPORTC_(0)
    , prevPORTD_(0)
    , interruptsEnabled_(false)
    , lastLoadReport_us_(0)
    , loadReportInterval_us_(20000.0)  // Report každých 20ms
{
    logFile_.open(logFile);
    if (!logFile_.is_open()) {
        std::cerr << "Failed to open log file: " << logFile << std::endl;
    }
    
    // Otevření samostatného logu pro zatížení motorů
    std::string loadLogName = logFile.substr(0, logFile.find_last_of('.')) + "_load.log";
    loadLogFile_.open(loadLogName);
    if (!loadLogFile_.is_open()) {
        std::cerr << "Failed to open load log file: " << loadLogName << std::endl;
    } else {
        loadLogFile_ << "Motor Load Analysis Log" << std::endl;
        loadLogFile_ << "Time (ms), Pin, Motor Type, Load (%), Pulse Width (us), Period (us)" << std::endl;
        loadLogFile_ << "========================================================================" << std::endl;
    }
    
    // Hlavička logu
    logFile_ << "AVR Timer1 Simulation Log" << std::endl;
    logFile_ << "CPU Frequency: " << cpuFrequency_ << " Hz" << std::endl;
    logFile_ << "======================================" << std::endl;
    logFile_ << std::endl;
}

AVRTimerSimulator::~AVRTimerSimulator() {
    if (logFile_.is_open()) {
        logFile_.close();
    }
    if (loadLogFile_.is_open()) {
        loadLogFile_.close();
    }
}

void AVRTimerSimulator::registerCompareMatchA_ISR(std::function<void()> callback) {
    compareMatchA_ISR_ = callback;
}

void AVRTimerSimulator::registerOverflow_ISR(std::function<void()> callback) {
    overflow_ISR_ = callback;
}

void AVRTimerSimulator::registerCompareMatchB_ISR(std::function<void()> callback) {
    compareMatchB_ISR_ = callback;
}

void AVRTimerSimulator::setCPUFrequency(uint32_t freq_hz) {
    cpuFrequency_ = freq_hz;
}

uint16_t AVRTimerSimulator::getPrescaler() {
    uint8_t cs_bits = AVRSim::TCCR1B & 0x07;
    switch (cs_bits) {
        case 0: return 0;      // Timer stopped
        case 1: return 1;      // No prescaling
        case 2: return 8;
        case 3: return 64;
        case 4: return 256;
        case 5: return 1024;
        default: return 0;
    }
}

void AVRTimerSimulator::tick() {
    cycleCount_++;
    
    uint16_t prescaler = getPrescaler();
    if (prescaler == 0) return;  // Timer stopped
    
    // Inkrementace prescaler counteru
    prescalerCounter_++;
    if (prescalerCounter_ >= prescaler) {
        prescalerCounter_ = 0;
        
        // Uložení předchozí hodnoty TCNT1 pro detekci overflow
        uint16_t prevTCNT1 = AVRSim::TCNT1;
        
        // Inkrementace Timer1
        AVRSim::TCNT1++;
        
        // Detekce overflow (65535 -> 0)
        if (prevTCNT1 == 0xFFFF && AVRSim::TCNT1 == 0) {
            AVRSim::TIFR1 |= (1 << AVRSim::TOV1);
        }
        
        // Detekce Compare Match A
        if (AVRSim::TCNT1 == AVRSim::OCR1A) {
            AVRSim::TIFR1 |= (1 << AVRSim::OCF1A);
        }
        
        // Detekce Compare Match B
        if (AVRSim::TCNT1 == AVRSim::OCR1B) {
            AVRSim::TIFR1 |= (1 << AVRSim::OCF1B);
        }
    }
    
    // Kontrola přerušení
    checkInterrupts();
    
    // Detekce změn pinů
    if (AVRSim::PORTB != prevPORTB_) {
        for (uint8_t i = 0; i < 8; i++) {
            bool prev = (prevPORTB_ >> i) & 1;
            bool curr = (AVRSim::PORTB >> i) & 1;
            if (prev != curr) {
                logPinChange('B', i, curr);
            }
        }
        prevPORTB_ = AVRSim::PORTB;
    }
    
    if (AVRSim::PORTC != prevPORTC_) {
        for (uint8_t i = 0; i < 8; i++) {
            bool prev = (prevPORTC_ >> i) & 1;
            bool curr = (AVRSim::PORTC >> i) & 1;
            if (prev != curr) {
                logPinChange('C', i, curr);
            }
        }
        prevPORTC_ = AVRSim::PORTC;
    }
    
    if (AVRSim::PORTD != prevPORTD_) {
        for (uint8_t i = 0; i < 8; i++) {
            bool prev = (prevPORTD_ >> i) & 1;
            bool curr = (AVRSim::PORTD >> i) & 1;
            if (prev != curr) {
                logPinChange('D', i, curr);
            }
        }
        prevPORTD_ = AVRSim::PORTD;
    }
}

void AVRTimerSimulator::checkInterrupts() {
    // Kontrola, zda jsou interrupty globálně povoleny
    if (!(AVRSim::SREG & 0x80)) return;
    
    // Compare Match A
    if ((AVRSim::TIFR1 & (1 << AVRSim::OCF1A)) && 
        (AVRSim::TIMSK1 & (1 << AVRSim::OCIE1A))) {
        AVRSim::TIFR1 &= ~(1 << AVRSim::OCF1A);  // Clear flag
        if (compareMatchA_ISR_) {
            compareMatchA_ISR_();
        }
    }
    
    // Compare Match B
    if ((AVRSim::TIFR1 & (1 << AVRSim::OCF1B)) && 
        (AVRSim::TIMSK1 & (1 << AVRSim::OCIE1B))) {
        AVRSim::TIFR1 &= ~(1 << AVRSim::OCF1B);  // Clear flag
        if (compareMatchB_ISR_) {
            compareMatchB_ISR_();
        }
    }
    
    // Overflow
    if ((AVRSim::TIFR1 & (1 << AVRSim::TOV1)) && 
        (AVRSim::TIMSK1 & (1 << AVRSim::TOIE1))) {
        AVRSim::TIFR1 &= ~(1 << AVRSim::TOV1);  // Clear flag
        if (overflow_ISR_) {
            overflow_ISR_();
        }
    }
}

void AVRTimerSimulator::logPinChange(char port, uint8_t pin, bool state) {
    double time_us = (double)cycleCount_ / cpuFrequency_ * 1000000.0;
    logFile_ << std::fixed << std::setprecision(3);
    logFile_ << "[" << std::setw(12) << time_us << " us] ";
    logFile_ << "PORT" << port << "." << (int)pin << " = " << (state ? "HIGH" : "LOW");
    logFile_ << "  (TCNT1=" << AVRSim::TCNT1 << ")";
    logFile_ << std::endl;
    
    // Převod PORT.pin na Arduino pin číslo
    uint8_t arduinoPin = 0;
    if (port == 'D') {
        arduinoPin = pin;  // PORTD.0-7 = Arduino 0-7
    } else if (port == 'B') {
        arduinoPin = pin + 8;  // PORTB.0-5 = Arduino 8-13
    } else if (port == 'C') {
        arduinoPin = pin + 14;  // PORTC.0-5 = Arduino 14-19 (A0-A5)
    }
    
    // Aktualizace monitoru pinu
    updatePinMonitor(arduinoPin, state);
}


void AVRTimerSimulator::logTimerState() {
    double time_us = (double)cycleCount_ / cpuFrequency_ * 1000000.0;
    logFile_ << std::fixed << std::setprecision(3);
    logFile_ << "[" << std::setw(12) << time_us << " us] ";
    logFile_ << "TCNT1=" << AVRSim::TCNT1 
             << ", OCR1A=" << AVRSim::OCR1A
             << ", OCR1B=" << AVRSim::OCR1B;
    logFile_ << std::endl;
}

void AVRTimerSimulator::simulate(double duration_seconds, double timestep_us) {
    uint64_t totalCycles = (uint64_t)(duration_seconds * cpuFrequency_);
    uint64_t cycles_per_step = (uint64_t)(timestep_us * cpuFrequency_ / 1000000.0);
    
    if (cycles_per_step == 0) cycles_per_step = 1;
    
    logFile_ << "Starting simulation for " << duration_seconds << " seconds" << std::endl;
    logFile_ << "Total cycles: " << totalCycles << std::endl;
    logFile_ << "Timestep: " << timestep_us << " us (" << cycles_per_step << " cycles)" << std::endl;
    logFile_ << "======================================" << std::endl;
    logFile_ << std::endl;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    uint64_t nextReport = totalCycles / 10;  // Report every 10%
    uint64_t reportCounter = 0;
    int test0 = 0;
    for (uint64_t i = 0; i < totalCycles; i += cycles_per_step) {
        
        for (uint64_t j = 0; j < cycles_per_step; j++) {
            tick();
            
            test0 = ((test0 +1) % 1000);
            if (test0 == 0){
                logFile_ << "  (TCNT1=" << AVRSim::TCNT1 << ")" << std::endl;
            }
        }
        
        // Periodické logování zatížení motorů
        logMotorLoads();
        
        // Progress report
        if (cycleCount_ >= nextReport * (reportCounter + 1)) {
            reportCounter++;
            std::cout << "Simulation progress: " << (reportCounter * 10) << "%" << std::endl;
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    logFile_ << std::endl;
    logFile_ << "======================================" << std::endl;
    logFile_ << "Simulation completed in " << duration.count() << " ms" << std::endl;
    logFile_ << "Total simulated time: " << duration_seconds << " seconds" << std::endl;
    logFile_ << "Total cycles: " << cycleCount_ << std::endl;
    
    std::cout << "Simulation completed in " << duration.count() << " ms" << std::endl;
    std::cout << "Log saved to file" << std::endl;
}

// Arduino-like digitalWrite funkce pro simulaci
// Pin mapping: 0-7 = PORTD, 8-13 = PORTB, 14-19 = PORTC (A0-A5)
void digitalWrite(uint8_t pin, uint8_t value) {
    if (pin <= 7) {
        // PORTD 0-7
        if (value) {
            AVRSim::PORTD |= (1 << pin);
        } else {
            AVRSim::PORTD &= ~(1 << pin);
        }
    } else if (pin <= 13) {
        // PORTB 0-5 (pins 8-13)
        uint8_t bit = pin - 8;
        if (value) {
            AVRSim::PORTB |= (1 << bit);
        } else {
            AVRSim::PORTB &= ~(1 << bit);
        }
    } else if (pin <= 19) {
        // PORTC 0-5 (pins 14-19, A0-A5)
        uint8_t bit = pin - 14;
        if (value) {
            AVRSim::PORTC |= (1 << bit);
        } else {
            AVRSim::PORTC &= ~(1 << bit);
        }
    }
}

// Konfigurace typu motoru na pinu
void AVRTimerSimulator::configurePin(uint8_t arduinoPin, MotorType type) {
    if (pinMonitors_.find(arduinoPin) == pinMonitors_.end()) {
        pinMonitors_[arduinoPin] = PinMonitor();
    }
    pinMonitors_[arduinoPin].motorType = type;
    
    const char* typeName = "";
    switch(type) {
        case MotorType::SERVO: typeName = "SERVO"; break;
        case MotorType::DC_MOTOR: typeName = "DC_MOTOR"; break;
        case MotorType::STEPPER: typeName = "STEPPER"; break;
        default: typeName = "NONE"; break;
    }
    
    std::cout << "Configured pin " << (int)arduinoPin << " as " << typeName << std::endl;
}

// Převod Arduino pin čísla na PORT a pin bit
uint8_t AVRTimerSimulator::arduinoPinToPort(uint8_t arduinoPin, uint8_t& portPin) {
    if (arduinoPin <= 7) {
        portPin = arduinoPin;
        return 'D';
    } else if (arduinoPin <= 13) {
        portPin = arduinoPin - 8;
        return 'B';
    } else if (arduinoPin <= 19) {
        portPin = arduinoPin - 14;
        return 'C';
    }
    return 0;
}

// Aktualizace sledování pinu při změně stavu
void AVRTimerSimulator::updatePinMonitor(uint8_t arduinoPin, bool state) {
    if (pinMonitors_.find(arduinoPin) == pinMonitors_.end()) {
        return;  // Pin není konfigurován
    }
    
    PinMonitor& monitor = pinMonitors_[arduinoPin];
    uint64_t currentTime_us = (cycleCount_ * 1000000ULL) / cpuFrequency_;
    
    if (state && !monitor.currentState) {
        // Nábězná hrana (LOW -> HIGH)
        if (monitor.lastRisingEdge_us > 0) {
            // Výpočet periody (čas mezi dvěma nábězněmi hranami)
            monitor.lastPeriod_us = currentTime_us - monitor.lastRisingEdge_us;
        }
        monitor.lastRisingEdge_us = currentTime_us;
        
    } else if (!state && monitor.currentState) {
        // Sestupná hrana (HIGH -> LOW)
        monitor.lastFallingEdge_us = currentTime_us;
        
        // Výpočet šířky pulzu
        if (monitor.lastRisingEdge_us > 0) {
            monitor.lastPulseWidth_us = currentTime_us - monitor.lastRisingEdge_us;
            
            // Výpočet zatížení podle typu motoru
            switch(monitor.motorType) {
                case MotorType::SERVO:
                    monitor.currentLoad = calculateServoLoad(monitor);
                    break;
                case MotorType::DC_MOTOR:
                    monitor.currentLoad = calculateDCLoad(monitor);
                    break;
                case MotorType::STEPPER:
                    // TODO: Implementace pro stepper motor
                    monitor.currentLoad = 0.0;
                    break;
                default:
                    monitor.currentLoad = 0.0;
                    break;
            }
        }
    }
    
    monitor.currentState = state;
    monitor.lastChangeTime_us = currentTime_us;
}

// Výpočet zatížení serva: 1ms = -100%, 1.5ms = 0%, 2ms = +100%
double AVRTimerSimulator::calculateServoLoad(const PinMonitor& monitor) {
    double pulse_us = monitor.lastPulseWidth_us;
    
    // Servo signály jsou typicky 1000-2000 us
    // 1000 us = -100% (plně doleva/zpět)
    // 1500 us = 0% (střed)
    // 2000 us = +100% (plně doprava/vpřed)
    
    if (pulse_us < 500 || pulse_us > 2500) {
        return 0.0;  // Mimo platný rozsah
    }
    
    double load = ((pulse_us - 1500.0) / 500.0) * 100.0;
    
    // Omezení na -100% až +100%
    if (load < -100.0) load = -100.0;
    if (load > 100.0) load = 100.0;
    
    return load;
}

// Výpočet zatížení DC motoru: duty cycle v procentech
double AVRTimerSimulator::calculateDCLoad(const PinMonitor& monitor) {
    if (monitor.lastPeriod_us <= 0) {
        return 0.0;  // Ještě nemáme kompletní periodu
    }
    
    // Duty cycle = (HIGH time / period) * 100%
    double dutyCycle = (monitor.lastPulseWidth_us / monitor.lastPeriod_us) * 100.0;
    
    // Omezení na 0-100%
    if (dutyCycle < 0.0) dutyCycle = 0.0;
    if (dutyCycle > 100.0) dutyCycle = 100.0;
    
    return dutyCycle;
}

// Logování zatížení všech konfigurovaných motorů
void AVRTimerSimulator::logMotorLoads() {
    if (!loadLogFile_.is_open()) return;
    
    uint64_t currentTime_us = (cycleCount_ * 1000000ULL) / cpuFrequency_;
    
    // Kontrola, zda je čas na další report
    if (currentTime_us - lastLoadReport_us_ < loadReportInterval_us_) {
        return;
    }
    
    lastLoadReport_us_ = currentTime_us;
    double time_ms = currentTime_us / 1000.0;
    
    for (const auto& pair : pinMonitors_) {
        uint8_t arduinoPin = pair.first;
        const PinMonitor& monitor = pair.second;
        
        if (monitor.motorType == MotorType::NONE) continue;
        
        const char* typeName = "";
        switch(monitor.motorType) {
            case MotorType::SERVO: typeName = "SERVO"; break;
            case MotorType::DC_MOTOR: typeName = "DC"; break;
            case MotorType::STEPPER: typeName = "STEPPER"; break;
            default: typeName = "UNKNOWN"; break;
        }
        
        loadLogFile_ << std::fixed << std::setprecision(3);
        loadLogFile_ << time_ms << ", ";
        loadLogFile_ << "Pin" << (int)arduinoPin << ", ";
        loadLogFile_ << typeName << ", ";
        loadLogFile_ << std::setprecision(1) << monitor.currentLoad << ", ";
        loadLogFile_ << std::setprecision(1) << monitor.lastPulseWidth_us << ", ";
        loadLogFile_ << std::setprecision(1) << monitor.lastPeriod_us;
        loadLogFile_ << std::endl;
    }
}
