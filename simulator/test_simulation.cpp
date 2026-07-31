// Příklad použití simulátoru
#include "avr_simulator.h"
#include "avr_mock.h"

// Include vašich ovladačů
#include "motor_control/servo_control.h"
#include "motor_control/dc_control.h"
#include "internals/timer_control.h"

// ISR funkce z vašich souborů
extern void OnTimer1CompareMatchServo();
extern void OnTimer1OwerflowServo();
extern void OnTimer1CompareMatchDC();
extern void OnTimer1OwerflowDC();
extern void OnTimer1OwerflowDC();
void onOverFlow(){
    OnTimer1OwerflowServo();
    OnTimer1OwerflowDC();
}

void onCompareMatch(){
    OnTimer1CompareMatchServo();
    OnTimer1CompareMatchDC();
}

int main() {
    std::cout << "AVR Timer Simulator - Motor Control Test" << std::endl;
    std::cout << "=========================================" << std::endl;
    
    // Vytvoření simulátoru
    AVRTimerSimulator simulator("motor_simulation.log");
    
    // Registrace ISR callbacků
    simulator.registerCompareMatchB_ISR(onCompareMatch);
    simulator.registerOverflow_ISR(onOverFlow);
    
    simulator.configurePin(9,MotorType::DC_MOTOR);
    simulator.configurePin(10,MotorType::DC_MOTOR);
    simulator.configurePin(11,MotorType::DC_MOTOR);

    std::cout << "\nInitializing motor controllers..." << std::endl;
    
    // Inicializace Timer1
    TimerControl::setup_Timers();
    
    enum testI  { SERVO, DC  };
    testI testi = DC;
    if (testi == SERVO){
    std::cout << "Simulating SERVO motors" << std::endl;
        // Vytvoření servo motorů
        ServoControl servo1(9);   // Pin 9 (PORTB.1)
        ServoControl servo2(10);  // Pin 10 (PORTB.2)
        ServoControl servo3(11);  // Pin 11 (PORTB.3)      

        // Nastavení pozic
        servo1.setTarget(0);      // Střed
        servo2.setTarget(500);    // +500
        servo3.setTarget(-500);   // -500
    } else if (testi == DC){
        std::cout << "Simulating dc motors" << std::endl;
        DCControl dc1(9,1);
        DCControl dc2(10,1);
        DCControl dc3(11,1);
        dc1.setTarget(000);
        dc2.setTarget(500);
        dc3.setTarget(900);
    }

    

    
    
    std::cout << "  Servo 1 (pin 9):  position = 0" << std::endl;
    std::cout << "  Servo 2 (pin 10): position = +500" << std::endl;
    std::cout << "  Servo 3 (pin 11): position = -500" << std::endl;
    
    std::cout << "\nStarting simulation..." << std::endl;
    
    // Simulace 100ms (dostatečné pro 5 cyklů všech motorů = 5 × 20ms)
    std::cout << "\n--- Test: 100 ms simulation ---" << std::endl;
    simulator.simulate(0.3, 100.0);  // 100ms, timestep 100 µs
    
    std::cout << "\nSimulation complete! Check motor_simulation.log for results." << std::endl;
    std::cout << "You should see PWM pulses on pins 9, 10, and 11." << std::endl;
    
    std::cout << "\nSimulation complete! Check motor_simulation.log for results." << std::endl;
    
    return 0;
}
