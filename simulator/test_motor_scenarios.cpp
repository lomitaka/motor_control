/**
 * Komplexní testovací scénáře pro různé typy motorů
 * 
 * Testuje:
 * - Servo motory: plynulé přechody, skoky, extrémy
 * - DC motory: rampy, PWM, náhlé změny
 * - Stepper motory: (příprava na budoucnost)
 */

#include "avr_simulator.h"
#include "../src/internals/timer_control.h"
#include "../include/motor_control/servo_control.h"
#include "../include/motor_control/dc_control.h"
#include <iostream>
#include <thread>
#include <chrono>

// Forward declarations ISR funkcí
void OnTimer1CompareMatchServo();
void OnTimer1OwerflowServo();
void OnTimer1CompareMatchDC();
void OnTimer1OwerflowDC();

void test_servo_smooth_transitions(AVRTimerSimulator& simulator) {
    std::cout << "\n=== TEST 1: Servo - Plynulé přechody ===" << std::endl;
    std::cout << "Simulace: 3 serva, plynulý přechod z -90° -> 0° -> +90°" << std::endl;
    
    // Konfigurace pinů jako servo motory
    simulator.configurePin(9, MotorType::SERVO);   // Servo 1 na pinu 9
    simulator.configurePin(10, MotorType::SERVO);  // Servo 2 na pinu 10
    simulator.configurePin(11, MotorType::SERVO);  // Servo 3 na pinu 11
    
    // Inicializace Timer1 pro servo
    TimerControl::Timer1_Init();
    TimerControl::setup_Timers();
    
    // Registrace servo ISR
    simulator.registerCompareMatchA_ISR(OnTimer1CompareMatchServo);
    simulator.registerOverflow_ISR(OnTimer1OwerflowServo);
    
    // Vytvoření 3 servo motorů
    ServoControl servo1(9);   // Pin 9 = PORTB.1
    ServoControl servo2(10);  // Pin 10 = PORTB.2
    ServoControl servo3(11);  // Pin 11 = PORTB.3
    
    // Plynulý přechod: -90° -> 0° -> +90°
    std::cout << "Fáze 1: Pozice -90° (1000us pulz)" << std::endl;
    servo1.setTarget(-90);
    servo2.setTarget(-90);
    servo3.setTarget(-90);
    simulator.simulate(0.5);  // 500ms
    
    std::cout << "Fáze 2: Pozice 0° (1500us pulz)" << std::endl;
    servo1.setTarget(0);
    servo2.setTarget(0);
    servo3.setTarget(0);
    simulator.simulate(0.5);  // 500ms
    
    std::cout << "Fáze 3: Pozice +90° (2000us pulz)" << std::endl;
    servo1.setTarget(90);
    servo2.setTarget(90);
    servo3.setTarget(90);
    simulator.simulate(0.5);  // 500ms
    
    std::cout << "✓ Test dokončen: 1.5 sekund simulace" << std::endl;
}

void test_servo_sharp_jumps(AVRTimerSimulator& simulator) {
    std::cout << "\n=== TEST 2: Servo - Náhlé skoky ===" << std::endl;
    std::cout << "Simulace: Skoky z extrému na extrém (-90° <-> +90°)" << std::endl;
    
    simulator.configurePin(9, MotorType::SERVO);
    
    TimerControl::Timer1_Init();
    TimerControl::setup_Timers();
    simulator.registerCompareMatchA_ISR(OnTimer1CompareMatchServo);
    simulator.registerOverflow_ISR(OnTimer1OwerflowServo);
    
    ServoControl servo(9);
    
    std::cout << "Skok 1: -90° -> +90°" << std::endl;
    servo.setTarget(-90);
    simulator.simulate(0.2);
    servo.setTarget(90);
    simulator.simulate(0.2);
    
    std::cout << "Skok 2: +90° -> -90°" << std::endl;
    servo.setTarget(-90);
    simulator.simulate(0.2);
    
    std::cout << "Skok 3: -90° -> 0°" << std::endl;
    servo.setTarget(0);
    simulator.simulate(0.2);
    
    std::cout << "✓ Test dokončen: 0.8 sekund simulace" << std::endl;
}

void test_servo_incremental(AVRTimerSimulator& simulator) {
    std::cout << "\n=== TEST 3: Servo - Postupné kroky ===" << std::endl;
    std::cout << "Simulace: Postupný přírůstek po 10° od -90° do +90°" << std::endl;
    
    simulator.configurePin(9, MotorType::SERVO);
    
    TimerControl::Timer1_Init();
    TimerControl::setup_Timers();
    simulator.registerCompareMatchA_ISR(OnTimer1CompareMatchServo);
    simulator.registerOverflow_ISR(OnTimer1OwerflowServo);
    
    ServoControl servo(9);
    
    for (int angle = -90; angle <= 90; angle += 10) {
        std::cout << "  Pozice: " << angle << "°" << std::endl;
        servo.setTarget(angle);
        simulator.simulate(0.1);  // 100ms na každou pozici
    }
    
    std::cout << "✓ Test dokončen: " << (19 * 0.1) << " sekund simulace" << std::endl;
}

void test_dc_motor_pwm_sweep(AVRTimerSimulator& simulator) {
    std::cout << "\n=== TEST 4: DC Motor - PWM rampa ===" << std::endl;
    std::cout << "Simulace: Postupné zvyšování rychlosti 0% -> 100%" << std::endl;
    
    // Konfigurace pinů jako DC motory
    simulator.configurePin(5, MotorType::DC_MOTOR);   // PWM pin 5
    simulator.configurePin(6, MotorType::DC_MOTOR);   // PWM pin 6
    
    TimerControl::Timer1_Init();
    TimerControl::setup_Timers();
    simulator.registerCompareMatchA_ISR(OnTimer1CompareMatchDC);
    simulator.registerOverflow_ISR(OnTimer1OwerflowDC);
    
    DCControl motor1(5, true);   // Pin 5, dopředu
    DCControl motor2(6, false);  // Pin 6, dozadu
    
    // Rampa 0% -> 100% po 10% krocích
    for (int speed = 0; speed <= 100; speed += 10) {
        std::cout << "  Rychlost: " << speed << "%" << std::endl;
        motor1.setTarget(speed);
        motor2.setTarget(speed);
        simulator.simulate(0.2);  // 200ms na každý krok
    }
    
    std::cout << "✓ Test dokončen: " << (11 * 0.2) << " sekund simulace" << std::endl;
}

void test_dc_motor_sharp_changes(AVRTimerSimulator& simulator) {
    std::cout << "\n=== TEST 5: DC Motor - Náhlé změny ===" << std::endl;
    std::cout << "Simulace: Skoky 0% -> 100% -> 50% -> 0%" << std::endl;
    
    simulator.configurePin(5, MotorType::DC_MOTOR);
    
    TimerControl::Timer1_Init();
    TimerControl::setup_Timers();
    simulator.registerCompareMatchA_ISR(OnTimer1CompareMatchDC);
    simulator.registerOverflow_ISR(OnTimer1OwerflowDC);
    
    DCControl motor(5, true);
    
    std::cout << "Fáze 1: 0% (stop)" << std::endl;
    motor.setTarget(0);
    simulator.simulate(0.3);
    
    std::cout << "Fáze 2: 100% (plný výkon - NÁHLÝ START!)" << std::endl;
    motor.setTarget(100);
    simulator.simulate(0.5);
    
    std::cout << "Fáze 3: 50% (polovina)" << std::endl;
    motor.setTarget(50);
    simulator.simulate(0.5);
    
    std::cout << "Fáze 4: 0% (náhlé zastavení)" << std::endl;
    motor.setTarget(0);
    simulator.simulate(0.3);
    
    std::cout << "✓ Test dokončen: 1.6 sekund simulace" << std::endl;
}

void test_dc_motor_direction_change(AVRTimerSimulator& simulator) {
    std::cout << "\n=== TEST 6: DC Motor - Změna směru ===" << std::endl;
    std::cout << "Simulace: Dopředu -> Stop -> Dozadu" << std::endl;
    
    simulator.configurePin(5, MotorType::DC_MOTOR);
    simulator.configurePin(6, MotorType::DC_MOTOR);
    
    TimerControl::Timer1_Init();
    TimerControl::setup_Timers();
    simulator.registerCompareMatchA_ISR(OnTimer1CompareMatchDC);
    simulator.registerOverflow_ISR(OnTimer1OwerflowDC);
    
    DCControl motorForward(5, true);   // Dopředu na pin 5
    DCControl motorBackward(6, false); // Dozadu na pin 6
    
    std::cout << "Fáze 1: Dopředu 80%" << std::endl;
    motorForward.setTarget(80);
    motorBackward.setTarget(0);
    simulator.simulate(0.5);
    
    std::cout << "Fáze 2: Stop (brzda)" << std::endl;
    motorForward.setTarget(0);
    motorBackward.setTarget(0);
    simulator.simulate(0.2);
    
    std::cout << "Fáze 3: Dozadu 80%" << std::endl;
    motorForward.setTarget(0);
    motorBackward.setTarget(80);
    simulator.simulate(0.5);
    
    std::cout << "Fáze 4: Stop" << std::endl;
    motorForward.setTarget(0);
    motorBackward.setTarget(0);
    simulator.simulate(0.2);
    
    std::cout << "✓ Test dokončen: 1.4 sekund simulace" << std::endl;
}

void test_dc_motor_fine_control(AVRTimerSimulator& simulator) {
    std::cout << "\n=== TEST 7: DC Motor - Jemné ovládání ===" << std::endl;
    std::cout << "Simulace: Malé změny rychlosti kolem 50%" << std::endl;
    
    simulator.configurePin(5, MotorType::DC_MOTOR);
    
    TimerControl::Timer1_Init();
    TimerControl::setup_Timers();
    simulator.registerCompareMatchA_ISR(OnTimer1CompareMatchDC);
    simulator.registerOverflow_ISR(OnTimer1OwerflowDC);
    
    DCControl motor(5, true);
    
    // Jemné ladění rychlosti kolem 50%
    int speeds[] = {45, 48, 50, 52, 55, 50, 45};
    for (int speed : speeds) {
        std::cout << "  Rychlost: " << speed << "%" << std::endl;
        motor.setTarget(speed);
        simulator.simulate(0.15);
    }
    
    std::cout << "✓ Test dokončen: " << (7 * 0.15) << " sekund simulace" << std::endl;
}

void test_mixed_motors(AVRTimerSimulator& simulator) {
    std::cout << "\n=== TEST 8: Kombinované použití ===" << std::endl;
    std::cout << "Simulace: 2 serva + 2 DC motory současně" << std::endl;
    
    // Konfigurace pinů
    simulator.configurePin(9, MotorType::SERVO);      // Servo 1
    simulator.configurePin(10, MotorType::SERVO);     // Servo 2
    simulator.configurePin(5, MotorType::DC_MOTOR);   // DC 1
    simulator.configurePin(6, MotorType::DC_MOTOR);   // DC 2
    
    TimerControl::Timer1_Init();
    TimerControl::setup_Timers();
    
    // Registrace obou typů ISR
    simulator.registerCompareMatchA_ISR(OnTimer1CompareMatchServo);
    simulator.registerOverflow_ISR(OnTimer1OwerflowServo);
    
    // Vytvoření motorů
    ServoControl servo1(9);
    ServoControl servo2(10);
    DCControl dc1(5);
    DCControl dc2(6);
    
    std::cout << "Fáze 1: Start - serva na 0°, DC na 30%" << std::endl;
    servo1.setTarget(0);
    servo2.setTarget(0);
    dc1.setTarget(30);
    dc2.setTarget(30);
    simulator.simulate(0.5);
    
    std::cout << "Fáze 2: Serva +45°, DC na 60%" << std::endl;
    servo1.setTarget(45);
    servo2.setTarget(-45);
    dc1.setTarget(60);
    dc2.setTarget(60);
    simulator.simulate(0.5);
    
    std::cout << "Fáze 3: Serva +90°, DC na 100%" << std::endl;
    servo1.setTarget(90);
    servo2.setTarget(-90);
    dc1.setTarget(100);
    dc2.setTarget(100);
    simulator.simulate(0.5);
    
    std::cout << "Fáze 4: Reset - serva 0°, DC stop" << std::endl;
    servo1.setTarget(0);
    servo2.setTarget(0);
    dc1.setTarget(0);
    dc2.setTarget(0);
    simulator.simulate(0.5);
    
    std::cout << "✓ Test dokončen: 2.0 sekund simulace" << std::endl;
}

void test_stress_test(AVRTimerSimulator& simulator) {
    std::cout << "\n=== TEST 9: Zátěžový test ===" << std::endl;
    std::cout << "Simulace: Rychlé změny po 50ms, 5 sekund" << std::endl;
    
    simulator.configurePin(9, MotorType::SERVO);
    simulator.configurePin(5, MotorType::DC_MOTOR);
    
    TimerControl::Timer1_Init();
    TimerControl::setup_Timers();
    simulator.registerCompareMatchA_ISR(OnTimer1CompareMatchServo);
    simulator.registerOverflow_ISR(OnTimer1OwerflowServo);
    
    ServoControl servo(9);
    DCControl motor(5);
    
    // 100 změn po 50ms = 5 sekund
    for (int i = 0; i < 100; i++) {
        int servoAngle = (i % 2 == 0) ? -45 : 45;
        int motorSpeed = 20 + (i % 80);
        
        servo.setTarget(servoAngle);
        motor.setTarget(motorSpeed);
        simulator.simulate(0.05);  // 50ms
        
        if (i % 20 == 0) {
            std::cout << "  Progress: " << (i * 100 / 100) << "%" << std::endl;
        }
    }
    
    std::cout << "✓ Test dokončen: 5.0 sekund simulace" << std::endl;
}

void test_stepper_placeholder(AVRTimerSimulator& simulator) {
    std::cout << "\n=== TEST 10: Stepper Motor (připraven pro budoucnost) ===" << std::endl;
    std::cout << "Konfigurace: 4 piny pro stepper motor" << std::endl;
    
    // Konfigurace pinů pro stepper (4 fáze)
    simulator.configurePin(2, MotorType::STEPPER);
    simulator.configurePin(3, MotorType::STEPPER);
    simulator.configurePin(4, MotorType::STEPPER);
    simulator.configurePin(5, MotorType::STEPPER);
    
    std::cout << "⚠ Stepper motor logika zatím není implementována" << std::endl;
    std::cout << "✓ Piny nakonfigurovány pro budoucí použití" << std::endl;
}

int main() {
    std::cout << "╔════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  AVR Motor Control - Comprehensive Test Scenarios     ║" << std::endl;
    std::cout << "║  Test různých typů motorů a scénářů použití           ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════╝" << std::endl;
    std::cout << std::endl;
    
    std::cout << "Vyberte test:" << std::endl;
    std::cout << "  1 - Servo: Plynulé přechody" << std::endl;
    std::cout << "  2 - Servo: Náhlé skoky" << std::endl;
    std::cout << "  3 - Servo: Postupné kroky" << std::endl;
    std::cout << "  4 - DC: PWM rampa" << std::endl;
    std::cout << "  5 - DC: Náhlé změny (0->100)" << std::endl;
    std::cout << "  6 - DC: Změna směru" << std::endl;
    std::cout << "  7 - DC: Jemné ovládání" << std::endl;
    std::cout << "  8 - Mix: Serva + DC současně" << std::endl;
    std::cout << "  9 - Stress: Rychlé změny (5s)" << std::endl;
    std::cout << " 10 - Stepper: Placeholder" << std::endl;
    std::cout << "  0 - Spustit všechny testy" << std::endl;
    std::cout << std::endl;
    std::cout << "Volba: ";
    
    int choice;
    std::cin >> choice;
    
    // Vytvoření simulátoru
    AVRTimerSimulator simulator("motor_test.log");
    
    switch(choice) {
        case 1: test_servo_smooth_transitions(simulator); break;
        case 2: test_servo_sharp_jumps(simulator); break;
        case 3: test_servo_incremental(simulator); break;
        case 4: test_dc_motor_pwm_sweep(simulator); break;
        case 5: test_dc_motor_sharp_changes(simulator); break;
        case 6: test_dc_motor_direction_change(simulator); break;
        case 7: test_dc_motor_fine_control(simulator); break;
        case 8: test_mixed_motors(simulator); break;
        case 9: test_stress_test(simulator); break;
        case 10: test_stepper_placeholder(simulator); break;
        case 0:
            // Spustit všechny testy postupně
            test_servo_smooth_transitions(simulator);
            test_servo_sharp_jumps(simulator);
            test_servo_incremental(simulator);
            test_dc_motor_pwm_sweep(simulator);
            test_dc_motor_sharp_changes(simulator);
            test_dc_motor_direction_change(simulator);
            test_dc_motor_fine_control(simulator);
            test_mixed_motors(simulator);
            test_stress_test(simulator);
            test_stepper_placeholder(simulator);
            break;
        default:
            std::cout << "Neplatná volba!" << std::endl;
            return 1;
    }
    
    std::cout << "\n╔════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  Test dokončen!                                        ║" << std::endl;
    std::cout << "║  Výstupy:                                              ║" << std::endl;
    std::cout << "║    - motor_test.log (změny pinů)                       ║" << std::endl;
    std::cout << "║    - motor_test_load.log (zatížení motorů)             ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════╝" << std::endl;
    
    return 0;
}
