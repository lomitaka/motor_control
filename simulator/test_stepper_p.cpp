#include "motor_control/stepper_positioning.h"
#include "internals/stepp_timer_control.h"
#include "internals/arduino.h"
 
#include "avr_simulator.h"
#include <iostream>
#include <iomanip>



void test_single_stepper_speeds(AVRTimerSimulator * sim) {
    
    std::cout << "\n=== Test 1: Single Stepper - Various Speeds ===" << std::endl;
    
    // Create stepper: STEP=pin 2, DIR=pin 3
    StepperPositioning motor1(2, 3);
    motor1.setAcceleration(1); //  acceleration
    motor1.setSpeed(200); // 500 steps/s² acceleration
    sim->configurePin(2, MotorType::STEPPER);
    sim->configurePin(3, MotorType::STEPPER);
    
   
    std::cout << "\nSetting 50 steps/s" << std::endl;



    motor1.setTargetTicks(-500) ;
    sim->simulate(1.0,4.0); // 2 seconds

    motor1.setTargetTicks(+200) ;
    sim->simulate(3.0,4.0); // 2 seconds

 
}

void test_multiple_steppers(AVRTimerSimulator * sim) {
    
    std::cout << "\n=== Test 2: Multiple Steppers ===" << std::endl;
    
    
    // Create 3 steppers on different pins
    StepperPositioning motor1(2, 3);   // STEP=2, DIR=3
    StepperPositioning motor2(4, 5);   // STEP=4, DIR=5
    StepperPositioning motor3(6, 7);   // STEP=6, DIR=7
    
    motor1.setAcceleration(1);
    motor2.setAcceleration(2);
    motor3.setAcceleration(1);
    
    sim->configurePin(2, MotorType::STEPPER);
    sim->configurePin(4, MotorType::STEPPER);
    sim->configurePin(6, MotorType::STEPPER);
    sim->configurePin(3, MotorType::NONE);
    sim->configurePin(5, MotorType::NONE);
    sim->configurePin(7, MotorType::NONE);
    
    
    // Set different speeds
    
    std::cout << "Setting speeds: Motor1=200 steps/s, Motor2=500 steps/s, Motor3=1000 steps/s" << std::endl;
    
    motor1.setTargetTicks(200);
    motor2.setTargetTicks(500);
    motor3.setTargetTicks(1000);
    
    
    // Simulate 5 seconds
    sim->simulate(5.0,4.0); // 2 seconds
    

    
}


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
    //test_multiple_steppers(&simulator);
    //test_direction_changes();
    //test_immediate_speed();
    destroyLogger();
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "All tests completed!" << std::endl;
    std::cout << "Check stepper_test*.log for pin activity" << std::endl;
    std::cout << "========================================" << std::endl;
    
    
    return 0;
}
