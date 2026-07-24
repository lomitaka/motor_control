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

int main() {
    std::cout << "AVR Timer Simulator - Motor Control Test" << std::endl;
    std::cout << "=========================================" << std::endl;
    
    // Vytvoření simulátoru
    AVRTimerSimulator simulator("motor_simulation.log");
    
    // Registrace ISR callbacků
    simulator.registerCompareMatchA_ISR(OnTimer1CompareMatchServo);
    simulator.registerOverflow_ISR(OnTimer1OwerflowServo);
    
    std::cout << "\nInitializing motor controllers..." << std::endl;
    
    // Inicializace Timer1
    TimerControl::setup_Timers();
    
    if (false){

        // Vytvoření servo motorů
        ServoControl servo1(9);   // Pin 9 (PORTB.1)
        ServoControl servo2(10);  // Pin 10 (PORTB.2)
        ServoControl servo3(11);  // Pin 11 (PORTB.3)      

        // Nastavení pozic
        servo1.setTarget(0);      // Střed
        servo2.setTarget(500);    // +500
        servo3.setTarget(-500);   // -500
    }

    
    DCControl dc1(9);
    dc1.setTarget(500);
    
    std::cout << "Motors initialized:" << std::endl;
    std::cout << "  Servo 1 (pin 9):  position = 0" << std::endl;
    std::cout << "  Servo 2 (pin 10): position = +500" << std::endl;
    std::cout << "  Servo 3 (pin 11): position = -500" << std::endl;
    
    std::cout << "\nStarting simulation..." << std::endl;
    
    // Simulace 100ms (dostatečné pro 5 cyklů všech motorů = 5 × 20ms)
    std::cout << "\n--- Test: 100 ms simulation ---" << std::endl;
    simulator.simulate(0.1, 100.0);  // 100ms, timestep 100 µs
    
    std::cout << "\nSimulation complete! Check motor_simulation.log for results." << std::endl;
    std::cout << "You should see PWM pulses on pins 9, 10, and 11." << std::endl;
    
    std::cout << "\nSimulation complete! Check motor_simulation.log for results." << std::endl;
    
    return 0;
}
