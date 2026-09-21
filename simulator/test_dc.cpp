#include "motor_control/dc_control.h"
#include "internals/timer_control.h"
#include "avr_simulator.h"

#include <iostream>

void OnTimer1CompareMatchDC();
void OnTimer1OwerflowDC();

int main() {
    std::cout << "DC motor simulation test" << std::endl;

    AVRTimerSimulator simulator("dc_simulation.log");
    simulator.configurePin(5, MotorType::DC_MOTOR);
    simulator.configurePin(4, MotorType::NONE);
    simulator.registerOverflow_ISR(OnTimer1OwerflowDC);
    simulator.registerCompareMatchB_ISR(OnTimer1CompareMatchDC);
    simulator.registerDebugLog([](uint32_t) {});

    DCControl motor(5, 4);
    motor.setTarget(100);
    simulator.simulate(0.2, 62.5);

    motor.setTarget(0);
    simulator.simulate(0.05, 62.5);

    simulator.logMotorLoads();
    std::cout << "DC simulation completed. Check dc_simulation.log and dc_simulation_load.log." << std::endl;
    return 0;
}
