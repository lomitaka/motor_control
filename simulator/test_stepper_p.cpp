#include "motor_control/stepper_positioning.h"
#include "internals/stepp_timer_control.h"
#include "internals/arduino.h"
 
#include "avr_simulator.h"
#include <iostream>
#include <iomanip>


/**
 * @file test_stepper.cpp
 * @brief Test program for stepper motor control with prescaler 256
 * 
 * Tests:
 * 1. Single stepper at various speeds (50-3000 steps/s)
 * 2. Multiple steppers (3 motors) at different speeds
 * 3. Acceleration/deceleration ramps
 * 4. Direction changes
 */


void test_single_stepper_speeds(AVRTimerSimulator * sim) {
    
    std::cout << "\n=== Test 1: Single Stepper - Various Speeds ===" << std::endl;
    
    // Create stepper: STEP=pin 2, DIR=pin 3
    StepperPositioning motor1(2, 3);
    motor1.setAcceleration(1); //  acceleration
    motor1.setSpeed(200); // 500 steps/s² acceleration
    sim->configurePin(2, MotorType::STEPPER);
    sim->configurePin(3, MotorType::STEPPER);
    
    // Test 50 steps/s for 2 seconds
    
    //std::cout << "Setting speed: 50 steps/s" << std::endl;
    
    //motor1.setTargetSpeed(50);
    
    //sim->simulate(2); // 2 seconds
    //std::cout << "Current speed: " << motor1.getCurrentSpeed() << " steps/s" << std::endl;
    
    
    // Test 500 steps/s for 2 seconds
    
    //std::cout << "\nSetting speed: 500 steps/s" << std::endl;
    
    //motor1.setTargetSpeed(500);
    
    
    //sim->simulate(2); // 2 seconds
    //std::cout << "Current speed: " << motor1.getCurrentSpeed() << " steps/s" << std::endl;
    
    
    // Test 3000 steps/s for 2 seconds
    
    std::cout << "\nSetting 50 steps/s" << std::endl;

    motor1.setTargetTicks(20) ;
    sim->simulate(5.0,4.0); // 2 seconds
    //motor1.setSpeed(100);
    //motor1.addTargetTicks(200) ;
    //sim->simulate(5.0,4.0); // 2 seconds

    //motor1.setTargetTicks(500) ;
    //sim->simulate(10.0,4.0); // 2 seconds
    //motor1.setTargetTicks(200) ;
    //motor1.setSpeed(1000) ;
    //sim->simulate(3.0,4.0); // 2 seconds

    
    //motor1.setTargetSpeed(3000);
    //motor1.setTargetTicks(3000);
    
    //sim->simulate(2); // 2 seconds
    //std::cout << "Current speed: " << motor1.getCurrentSpeed() << " steps/s" << std::endl;
    
    
    // Stop motor
    
    //std::cout << "\nStopping motor (target = 0 steps/s)" << std::endl;
    
    //motor1.setTargetSpeed(0);
    
    
    //sim->simulate(2); // 2 seconds
    //std::cout << "Current speed: " << motor1.getCurrentSpeed() << " steps/s" << std::endl;
    //std::cout << "Is moving: " << (motor1.isMoving() ? "Yes" : "No") << std::endl;
        
}

void test_multiple_steppers() {
    
    std::cout << "\n=== Test 2: Multiple Steppers ===" << std::endl;
    
    
    // Create 3 steppers on different pins
    StepperPositioning motor1(2, 3);   // STEP=2, DIR=3
    StepperPositioning motor2(4, 5);   // STEP=4, DIR=5
    StepperPositioning motor3(6, 7);   // STEP=6, DIR=7
    
    motor1.setAcceleration(500);
    motor2.setAcceleration(300);
    motor3.setAcceleration(800);
    
    
    AVRTimerSimulator sim;
    sim.configurePin(2, MotorType::STEPPER);
    sim.configurePin(4, MotorType::STEPPER);
    sim.configurePin(6, MotorType::STEPPER);
    sim.configurePin(3, MotorType::NONE);
    sim.configurePin(5, MotorType::NONE);
    sim.configurePin(7, MotorType::NONE);
    
    
    // Set different speeds
    
    std::cout << "Setting speeds: Motor1=200 steps/s, Motor2=500 steps/s, Motor3=1000 steps/s" << std::endl;
    
    motor1.setTargetTicks(200);
    motor2.setTargetTicks(500);
    motor3.setTargetTicks(1000);
    
    
    // Simulate 5 seconds
    sim.simulate(5);
    
    /*std::cout << "Final speeds:" << std::endl;
    std::cout << "  Motor1: " << motor1.getCurrentSpeed() << " steps/s (target: " << motor1.getTargetSpeed() << ")" << std::endl;
    std::cout << "  Motor2: " << motor2.getCurrentSpeed() << " steps/s (target: " << motor2.getTargetSpeed() << ")" << std::endl;
    std::cout << "  Motor3: " << motor3.getCurrentSpeed() << " steps/s (target: " << motor3.getTargetSpeed() << ")" << std::endl;
    */
    sim.logMotorLoads();
    
}
/*
void test_direction_changes() {
    
    std::cout << "\n=== Test 3: Direction Changes ===" << std::endl;
    
    
    StepperContinuous motor(2, 3);
    motor.setAcceleration(1000); // Fast acceleration for quick test
    
    
    AVRTimerSimulator sim;
    sim.configurePin(2, MotorType::STEPPER);
    sim.configurePin(3, MotorType::NONE);
    
    
    // Forward 500 steps/s
    
    std::cout << "Forward: 500 steps/s" << std::endl;
    
    motor.setTargetSpeed(500);
    
    
    sim.simulate(2000000); // 2 seconds
    std::cout << "Current: " << motor.getCurrentSpeed() << " steps/s" << std::endl;
    
    
    // Reverse -500 steps/s
    
    std::cout << "\nReverse: -500 steps/s" << std::endl;
    
    motor.setTargetSpeed(-500);
    
    
    sim.simulate(2000000); // 2 seconds
    std::cout << "Current: " << motor.getCurrentSpeed() << " steps/s" << std::endl;
    
    
    // Forward again 200 steps/s
    
    std::cout << "\nForward: 200 steps/s" << std::endl;
    
    motor.setTargetSpeed(200);
    
    
    sim.simulate(2000000); // 2 seconds
    std::cout << "Current: " << motor.getCurrentSpeed() << " steps/s" << std::endl;
    
    sim.logMotorLoads();
    
}

void test_immediate_speed() {
    
    std::cout << "\n=== Test 4: Immediate Speed (No Acceleration) ===" << std::endl;
    
    
    StepperContinuous motor(2, 3);
    
    
    AVRTimerSimulator sim;
    sim.configurePin(2, MotorType::STEPPER);
    sim.configurePin(3, MotorType::NONE);
    
    
    // Immediate jump to 2000 steps/s
    
    std::cout << "Immediate speed: 2000 steps/s" << std::endl;
    
    motor.setImmediateSpeed(2000);
    
    
    std::cout << "Current: " << motor.getCurrentSpeed() << " steps/s (should be 2000 immediately)" << std::endl;
    sim.simulate(1000000); // 1 second
    
    // Immediate stop
    std::cout << "\nImmediate stop" << std::endl;
    motor.setImmediateSpeed(0);
    std::cout << "Current: " << motor.getCurrentSpeed() << " steps/s (should be 0 immediately)" << std::endl;
    
    sim.logMotorLoads();
    
}

*/
void OnTimer1StepperPositioningOverflow();
void OnTimer1StepperPositioningOCRA();
void OnTimer1StepperPositioningOCRB();
void OnLoggerCalled();
DebugInfo GetDebugInfo();

std::ofstream logFile2_;
void openLogger(std::string logfile){
    logFile2_.open(logfile);
    if (!logFile2_.is_open()) {
        std::cerr << "Failed to open log file: " << logfile << std::endl;
    }
    logFile2_ << "time_us,current,target,remaining\n";
}

void destroyLogger(){
    logFile2_.close();
}

uint32_t hash = 0;
DebugInfo di_old  = {0,0,0};
void logDebugInfo(uint32_t time_us){
    DebugInfo di = GetDebugInfo();
    
    uint32_t hash2 = di.current_interval xor di.target_interval xor di.remainining_interval;
    if (hash2 == hash){return;}
    hash = hash2;
    
    logFile2_ << time_us << ",";
    logFile2_ << di_old.current_interval << ",";
    logFile2_ << di_old.target_interval << ",";
    logFile2_ << di_old.remainining_interval << '\n';

    
    logFile2_ << time_us+1 << ",";
    logFile2_ << di.current_interval << ",";
    logFile2_ << di.target_interval << ",";
    logFile2_ << di.remainining_interval << '\n';
    
    di_old = di;

}


struct Interval {
    uint8_t skip_count;//255 for full stop
    uint16_t remainder;

    // Elegantnější cesta: Přetížení operátoru <
    // Pozor: Operátor v C++ standardně bere jen pravou stranu, 
    // takže divider musíme předat jinak, nebo předpokládat fixní divider.
    // Zde ukázka s předpokladem stejného divideru pro obě struktury:
    bool operator<(const volatile Interval& other) const volatile {
        uint8_t sc1 = skip_count;
        uint8_t sc2 = other.skip_count;
        if (sc1 != sc2) return sc1 < sc2;
        
        uint16_t rem1 = remainder;
        uint16_t rem2 = other.remainder;
        return rem1 < rem2;
    }

    // Přetížení operátoru rovnosti (==)
    bool operator==(const volatile Interval& other) const volatile {
        return (skip_count == other.skip_count) && (remainder == other.remainder);
    }

    // Přetížení operátoru nerovnosti (!=)
    bool operator!=(const volatile Interval& other) const volatile {
        return (skip_count !=  other.skip_count) || (remainder != other.remainder);
    }
    
    bool operator>(const volatile Interval& other) const volatile {
    uint8_t sc1 = skip_count;
    uint8_t sc2 = other.skip_count;
    if (sc1 != sc2) return sc1 > sc2; // Větší než (>)
    
    uint16_t rem1 = remainder;
    uint16_t rem2 = other.remainder;
    return rem1 > rem2; // Větší než (>)
    }

    volatile Interval& operator=(const volatile Interval& other) volatile {
        // Bezpečně načteme hodnoty z 'other' do lokálních proměnných 
        // (pokud by 'other' byl náhodou také volatile)
        uint8_t sc = other.skip_count;
        uint16_t rem = other.remainder;

        // Zapíšeme je do našich volatile položek
        skip_count = sc;
        remainder = rem;

        // Operátor přiřazení v C++ standardně vrací referenci na sebe, 
        // abyste mohli řetězit přiřazení typu a = b = c;
        return *this;
    }

};



#define STOP 250
#define SLOW 9
#define VERY_SLOW 244
#define VERY_SLOW2 (uint32_t)244 << 16

void addToInterval2d(volatile Interval & target, volatile Interval & new_current, uint8_t modifier){
    //modifier -number from 1-10 for more aggresive interval update

    if (target > new_current){
        //need to add to new_current
        uint32_t new_curr = ((uint32_t)new_current.skip_count << 16) + new_current.remainder;
        //add 8 percent of new_current to new_curr
        new_curr = new_curr + modifier*(new_curr >> 3);
        //handle top (converts it into stop)target_speeds_stp_ps_
        if (new_current.skip_count > VERY_SLOW ) {new_curr = ((uint32_t)VERY_SLOW << 16);}
        new_current.remainder = new_curr & 0xFFFF;
        new_current.skip_count = new_curr >> 16;
        
        if (new_current > target){
            new_current.remainder = target.remainder;
            new_current.skip_count = target.skip_count;
        }
        
    }else if (target < new_current){
        uint32_t new_curr = ((uint32_t)new_current.skip_count << 16) + new_current.remainder;
        //add 8 percent of new_current to new_curr
        
        new_curr = new_curr - modifier*(new_curr >> 3);
        //lower protectin
        if (new_curr < 200) {new_curr = 200;}
        new_current.remainder = new_curr & 0xFFFF;
        new_current.skip_count = new_curr >> 16;
        
        if (new_current < target){
            new_current.remainder = target.remainder;
            new_current.skip_count = target.skip_count;
        }
    }
}


void addTest(){

    uint32_t targetn =  589824;
    uint32_t currentn =  13421774;
    volatile Interval target;
    target.skip_count = targetn / 65536;
    target.remainder = targetn % 65536;
    
    volatile Interval current;
    current.skip_count = currentn / 65536;
    current.remainder = currentn % 65536;

    for (int i = 0 ; i < 2000; i++){

        addToInterval2d(target, current, 1);
        uint32_t val =  (current.skip_count << 16)+ current.remainder;
        std::cout << val << std::endl;
    }
}



int main() {
    //addTest();
    //return 0;
    
    std::cout << "========================================" << std::endl;
    std::cout << "Stepper Motor Test with Prescaler 256" << std::endl;
    std::cout << "Timer: 62.5 kHz, ISR: 100 Hz (10ms)" << std::endl;
    std::cout << "========================================" << std::endl;
        
    // Vytvoření simulátoru
    AVRTimerSimulator simulator("motor_simulation.log");
    openLogger("motor_logs.log");
    // Registrace ISR callbacků
    //simulator.registerCompareMatchB_ISR(onCompareMatch);
    simulator.registerOverflow_ISR(OnTimer1StepperPositioningOverflow);
    simulator.registerCompareMatchA_ISR(OnTimer1StepperPositioningOCRA);
    simulator.registerCompareMatchB_ISR(OnTimer1StepperPositioningOCRB);
    simulator.registerDebugLog(logDebugInfo);
    
    

    test_single_stepper_speeds(&simulator);
    //test_multiple_steppers();
    //test_direction_changes();
    //test_immediate_speed();
    destroyLogger();
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "All tests completed!" << std::endl;
    std::cout << "Check stepper_test*.log for pin activity" << std::endl;
    std::cout << "========================================" << std::endl;
    
    
    return 0;
}
