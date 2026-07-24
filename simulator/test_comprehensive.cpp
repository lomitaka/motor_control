/**
 * Comprehensive Motor Control Test Suite
 * 
 * Tests typical and edge-case usage scenarios for:
 * - Servo motors (position control)
 * - DC motors (speed control)
 * 
 * Test Categories:
 * 1. Basic functionality - standard operation
 * 2. Gradual changes - ramp up/down speeds and positions
 * 3. Stress tests - sudden changes, extremes
 * 4. Real-world scenarios - combined movements
 */

#include "avr_simulator.h"
#include "avr_mock.h"

// Include motor controllers
#include "motor_control/servo_control.h"
#include "motor_control/dc_control.h"
#include "internals/timer_control.h"

// ISR functions
extern void OnTimer1CompareMatchServo();
extern void OnTimer1OwerflowServo();
extern void OnTimer1CompareMatchDC();
extern void OnTimer1OwerflowDC();

// Global simulator instance
AVRTimerSimulator* g_simulator = nullptr;

// Helper function to wait and simulate
void simulateFor(double duration_ms, const char* description = nullptr) {
    if (description) {
        std::cout << "\n  >> " << description << " (" << duration_ms << " ms)" << std::endl;
    }
    g_simulator->simulate(0.1, duration_ms);
}

// Print test header
void printTestHeader(const char* testName) {
    std::cout << "\n" << std::string(70, '=') << std::endl;
    std::cout << "TEST: " << testName << std::endl;
    std::cout << std::string(70, '=') << std::endl;
}

// ============================================================================
// SERVO MOTOR TESTS
// ============================================================================

void test_servo_basic_positions() {
    printTestHeader("Servo - Basic Positions");
    std::cout << "Testing standard servo positions: center, min, max" << std::endl;
    
    ServoControl servo(9);  // Pin 9 (PORTB.1)
    
    // Center position
    std::cout << "\n1. Center position (0)" << std::endl;
    servo.setImmediate(0);
    simulateFor(50, "Hold at center");
    
    // Maximum positive
    std::cout << "\n2. Maximum positive (+1000)" << std::endl;
    servo.setImmediate(1000);
    simulateFor(50, "Hold at +1000");
    
    // Maximum negative
    std::cout << "\n3. Maximum negative (-1000)" << std::endl;
    servo.setImmediate(-1000);
    simulateFor(50, "Hold at -1000");
    
    // Back to center
    std::cout << "\n4. Back to center (0)" << std::endl;
    servo.setImmediate(0);
    simulateFor(50, "Hold at center");
}

void test_servo_gradual_movement() {
    printTestHeader("Servo - Gradual Movement (Linear Ramp)");
    std::cout << "Testing smooth position changes with ramping" << std::endl;
    
    ServoControl servo(10);  // Pin 10 (PORTB.2)
    
    // Configure linear ramp - 1 second to reach target
    servo.configureRamp(1000);
    
    std::cout << "\n1. Gradual move from 0 to +800" << std::endl;
    servo.setImmediate(0);
    simulateFor(50, "Starting position");
    
    servo.setTarget(800);
    simulateFor(1200, "Ramping to +800");
    
    std::cout << "\n2. Gradual move from +800 to -800" << std::endl;
    servo.setTarget(-800);
    simulateFor(1200, "Ramping to -800");
    
    std::cout << "\n3. Gradual return to center" << std::endl;
    servo.setTarget(0);
    simulateFor(1200, "Ramping to 0");
}

void test_servo_step_sequence() {
    printTestHeader("Servo - Step Sequence");
    std::cout << "Testing incremental position changes" << std::endl;
    
    ServoControl servo(11);  // Pin 11 (PORTB.3)
    
    std::cout << "\nIncrementing position by 200 units:" << std::endl;
    for (int16_t pos = -1000; pos <= 1000; pos += 200) {
        std::cout << "  Position: " << pos << std::endl;
        servo.setImmediate(pos);
        simulateFor(100, nullptr);
    }
    
    std::cout << "\nDecrementing position by 250 units:" << std::endl;
    for (int16_t pos = 1000; pos >= -1000; pos -= 250) {
        std::cout << "  Position: " << pos << std::endl;
        servo.setImmediate(pos);
        simulateFor(100, nullptr);
    }
}

void test_servo_stress_sudden_changes() {
    printTestHeader("Servo - Stress Test: Sudden Changes");
    std::cout << "Testing extreme position jumps (not recommended in practice!)" << std::endl;
    
    ServoControl servo(9);  // Pin 9 (PORTB.1)
    
    std::cout << "\n1. Neutral to max positive (0 -> +1000)" << std::endl;
    servo.setImmediate(0);
    simulateFor(50, "Start at 0");
    servo.setImmediate(1000);
    simulateFor(50, "Jump to +1000");
    
    std::cout << "\n2. Max positive to max negative (+1000 -> -1000)" << std::endl;
    servo.setImmediate(-1000);
    simulateFor(50, "Jump to -1000");
    
    std::cout << "\n3. Rapid oscillation" << std::endl;
    for (int i = 0; i < 5; i++) {
        servo.setImmediate(1000);
        simulateFor(25, nullptr);
        servo.setImmediate(-1000);
        simulateFor(25, nullptr);
    }
    
    std::cout << "\n4. Return to safe center position" << std::endl;
    servo.setImmediate(0);
    simulateFor(50, "Stabilize at center");
}

// ============================================================================
// DC MOTOR TESTS
// ============================================================================

void test_dc_basic_speeds() {
    printTestHeader("DC Motor - Basic Speed Control");
    std::cout << "Testing standard speed settings: 0%, 25%, 50%, 75%, 100%" << std::endl;
    
    DCControl dc(12,1);  // Pin 12 (PORTB.4)
    
    std::cout << "\n1. Stop (0)" << std::endl;
    dc.setImmediate(0);
    simulateFor(100, "Motor stopped");
    
    std::cout << "\n2. 25% speed forward (+250)" << std::endl;
    dc.setImmediate(250);
    simulateFor(100, "Running at 25%");
    
    std::cout << "\n3. 50% speed forward (+500)" << std::endl;
    dc.setImmediate(500);
    simulateFor(100, "Running at 50%");
    
    std::cout << "\n4. 75% speed forward (+750)" << std::endl;
    dc.setImmediate(750);
    simulateFor(100, "Running at 75%");
    
    std::cout << "\n5. 100% speed forward (+1000)" << std::endl;
    dc.setImmediate(1000);
    simulateFor(100, "Running at 100%");
    
    std::cout << "\n6. Back to stop (0)" << std::endl;
    dc.setImmediate(0);
    simulateFor(100, "Motor stopped");
}

void test_dc_direction_changes() {
    printTestHeader("DC Motor - Direction Control");
    std::cout << "Testing forward/reverse operation" << std::endl;
    
    DCControl dc(13,1);  // Pin 13 (PORTB.5)
    
    std::cout << "\n1. Forward at 60% (+600)" << std::endl;
    dc.setImmediate(600);
    simulateFor(150, "Running forward");
    
    std::cout << "\n2. Stop before direction change" << std::endl;
    dc.setImmediate(0);
    simulateFor(50, "Stopped");
    
    std::cout << "\n3. Reverse at 60% (-600)" << std::endl;
    dc.setImmediate(-600);
    simulateFor(150, "Running reverse");
    
    std::cout << "\n4. Stop" << std::endl;
    dc.setImmediate(0);
    simulateFor(50, "Stopped");
    
    std::cout << "\n5. Forward at 40% (+400)" << std::endl;
    dc.setImmediate(400);
    simulateFor(150, "Running forward");
    
    std::cout << "\n6. Final stop" << std::endl;
    dc.setImmediate(0);
    simulateFor(50, "Stopped");
}

void test_dc_gradual_acceleration() {
    printTestHeader("DC Motor - Gradual Acceleration");
    std::cout << "Testing smooth speed ramp-up (soft start)" << std::endl;
    
    DCControl dc(12,1);  // Pin 12 (PORTB.4)
    
    dc.setImmediate(0);
    simulateFor(50, "Starting from stop");
    
    std::cout << "\nGradual acceleration from 0 to 1000:" << std::endl;
    for (int16_t speed = 0; speed <= 1000; speed += 100) {
        std::cout << "  Speed: " << speed << " (" << (speed/10) << "%)" << std::endl;
        dc.setImmediate(speed);
        simulateFor(200, nullptr);
    }
    
    simulateFor(200, "Hold at max speed");
}

void test_dc_gradual_deceleration() {
    printTestHeader("DC Motor - Gradual Deceleration");
    std::cout << "Testing smooth braking (soft stop)" << std::endl;
    
    DCControl dc(13,1);  // Pin 13 (PORTB.5)
    
    std::cout << "\nStarting at full speed" << std::endl;
    dc.setImmediate(1000);
    simulateFor(200, "Running at 100%");
    
    std::cout << "\nGradual deceleration from 1000 to 0:" << std::endl;
    for (int16_t speed = 1000; speed >= 0; speed -= 100) {
        std::cout << "  Speed: " << speed << " (" << (speed/10) << "%)" << std::endl;
        dc.setImmediate(speed);
        simulateFor(200, nullptr);
    }
    
    simulateFor(100, "Motor stopped");
}

void test_dc_stress_sudden_reverse() {
    printTestHeader("DC Motor - Stress Test: Sudden Direction Reversal");
    std::cout << "WARNING: This is NOT recommended for real hardware!" << std::endl;
    std::cout << "Testing immediate direction reversal under load" << std::endl;
    
    DCControl dc(12,1);  // Pin 12 (PORTB.4)
    
    std::cout << "\n1. Full speed forward (+1000)" << std::endl;
    dc.setImmediate(1000);
    simulateFor(100, "Running forward at 100%");
    
    std::cout << "\n2. SUDDEN REVERSE to -1000 (STRESS!)" << std::endl;
    dc.setImmediate(-1000);
    simulateFor(100, "Running reverse at 100%");
    
    std::cout << "\n3. SUDDEN FORWARD to +1000 (STRESS!)" << std::endl;
    dc.setImmediate(1000);
    simulateFor(100, "Running forward at 100%");
    
    std::cout << "\n4. Safe stop" << std::endl;
    dc.setImmediate(0);
    simulateFor(100, "Motor stopped");
}

void test_dc_pwm_duty_cycle_analysis() {
    printTestHeader("DC Motor - PWM Duty Cycle Analysis");
    std::cout << "Testing various speeds to analyze PWM characteristics" << std::endl;
    
    DCControl dc(9,1);  // Pin 9 (PORTB.1)
    
    int16_t test_speeds[] = {100, 250, 500, 750, 900, 1000};
    
    for (int16_t speed : test_speeds) {
        std::cout << "\nSpeed: " << speed << " (" << (speed/10) << "% duty cycle expected)" << std::endl;
        dc.setImmediate(speed);
        simulateFor(200, "Measure PWM");
    }
    
    dc.setImmediate(0);
    simulateFor(50, "Stop");
}

// ============================================================================
// COMBINED / REAL-WORLD SCENARIOS
// ============================================================================

void test_combined_servo_and_dc() {
    printTestHeader("Combined Test - Servo + DC Motor");
    std::cout << "Real-world scenario: Robotic arm with servo and drive motor" << std::endl;
    
    ServoControl servo(9);   // Pin 9 - Servo for arm position
    DCControl dc(12,1);        // Pin 12 - DC motor for drive
    
    std::cout << "\nScenario: Robot approaches object, positions arm, grabs it" << std::endl;
    
    std::cout << "\n1. Start driving forward at 50%" << std::endl;
    dc.setImmediate(500);
    servo.setImmediate(0);  // Arm neutral
    simulateFor(300, "Approaching");
    
    std::cout << "\n2. Slow down to 25%" << std::endl;
    dc.setImmediate(250);
    simulateFor(200, "Getting closer");
    
    std::cout << "\n3. Stop and position arm to -600" << std::endl;
    dc.setImmediate(0);
    servo.setImmediate(-600);  // Arm down
    simulateFor(300, "Positioning arm");
    
    std::cout << "\n4. Arm to +800 (simulating grab)" << std::endl;
    servo.setImmediate(800);
    simulateFor(300, "Grabbing");
    
    std::cout << "\n5. Return arm to neutral and drive back" << std::endl;
    servo.setImmediate(0);
    dc.setImmediate(-500);  // Reverse
    simulateFor(300, "Returning");
    
    std::cout << "\n6. Stop everything" << std::endl;
    dc.setImmediate(0);
    servo.setImmediate(0);
    simulateFor(100, "Mission complete");
}

void test_multiple_motors_coordination() {
    printTestHeader("Multiple Motors - Coordination Test");
    std::cout << "Testing multiple motors running simultaneously" << std::endl;
    
    ServoControl servo1(9);   // Pin 9
    ServoControl servo2(10);  // Pin 10
    DCControl dc1(12,1);        // Pin 12
    DCControl dc2(13,1);        // Pin 13
    
    std::cout << "\n1. Initialize all motors" << std::endl;
    servo1.setImmediate(0);
    servo2.setImmediate(0);
    dc1.setImmediate(0);
    dc2.setImmediate(0);
    simulateFor(100, "All at rest");
    
    std::cout << "\n2. Start all motors at different values" << std::endl;
    servo1.setImmediate(-500);
    servo2.setImmediate(500);
    dc1.setImmediate(600);
    dc2.setImmediate(-600);
    simulateFor(300, "All running");
    
    std::cout << "\n3. Change all simultaneously" << std::endl;
    servo1.setImmediate(800);
    servo2.setImmediate(-800);
    dc1.setImmediate(-400);
    dc2.setImmediate(400);
    simulateFor(300, "New positions/speeds");
    
    std::cout << "\n4. Sequential stop" << std::endl;
    dc1.setImmediate(0);
    simulateFor(100, "DC1 stopped");
    dc2.setImmediate(0);
    simulateFor(100, "DC2 stopped");
    servo1.setImmediate(0);
    simulateFor(100, "Servo1 centered");
    servo2.setImmediate(0);
    simulateFor(100, "Servo2 centered");
}

// ============================================================================
// MAIN TEST RUNNER
// ============================================================================

int main(int argc, char* argv[]) {
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║     COMPREHENSIVE MOTOR CONTROL TEST SUITE                         ║\n";
    std::cout << "║     AVR ATmega328p Simulator                                       ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════════════╝\n";
    
    // Determine log file name
    std::string logFile = "motor_comprehensive_test.log";
    if (argc > 1) {
        logFile = argv[1];
    }
    
    // Create simulator
    AVRTimerSimulator simulator(logFile.c_str());
    g_simulator = &simulator;
    
    // Register ISR callbacks for servo
    simulator.registerCompareMatchA_ISR(OnTimer1CompareMatchServo);
    simulator.registerOverflow_ISR(OnTimer1OwerflowServo);
    
    // Note: If you want to test DC motors with their ISRs, uncomment:
    // simulator.registerCompareMatchA_ISR(OnTimer1CompareMatchDC);
    // simulator.registerOverflow_ISR(OnTimer1OwerflowDC);
    
    // Initialize Timer1
    std::cout << "\nInitializing Timer1..." << std::endl;
    TimerControl::setup_Timers();
    std::cout << "Timer1 initialized successfully!\n" << std::endl;
    
    // Test selection (run all by default, or specify test number)
    int test_to_run = 0;  // 0 = all tests
    if (argc > 2) {
        test_to_run = std::atoi(argv[2]);
    }
    
    // ========================================================================
    // SERVO TESTS
    // ========================================================================
    
    if (test_to_run == 0 || test_to_run == 1) {
        test_servo_basic_positions();
    }
    
    if (test_to_run == 0 || test_to_run == 2) {
        test_servo_gradual_movement();
    }
    
    if (test_to_run == 0 || test_to_run == 3) {
        test_servo_step_sequence();
    }
    
    if (test_to_run == 0 || test_to_run == 4) {
        test_servo_stress_sudden_changes();
    }
    
    // ========================================================================
    // DC MOTOR TESTS
    // ========================================================================
    
    if (test_to_run == 0 || test_to_run == 5) {
        test_dc_basic_speeds();
    }
    
    if (test_to_run == 0 || test_to_run == 6) {
        test_dc_direction_changes();
    }
    
    if (test_to_run == 0 || test_to_run == 7) {
        test_dc_gradual_acceleration();
    }
    
    if (test_to_run == 0 || test_to_run == 8) {
        test_dc_gradual_deceleration();
    }
    
    if (test_to_run == 0 || test_to_run == 9) {
        test_dc_stress_sudden_reverse();
    }
    
    if (test_to_run == 0 || test_to_run == 10) {
        test_dc_pwm_duty_cycle_analysis();
    }
    
    // ========================================================================
    // COMBINED TESTS
    // ========================================================================
    
    if (test_to_run == 0 || test_to_run == 11) {
        test_combined_servo_and_dc();
    }
    
    if (test_to_run == 0 || test_to_run == 12) {
        test_multiple_motors_coordination();
    }
    
    // ========================================================================
    // SUMMARY
    // ========================================================================
    
    printTestHeader("TEST SUITE COMPLETE");
    std::cout << "\nResults logged to: " << logFile << std::endl;
    std::cout << "\nTo visualize the results, run:" << std::endl;
    std::cout << "  python3 plot_simulation.py " << logFile << std::endl;
    
    std::cout << "\nTo run individual tests:" << std::endl;
    std::cout << "  ./motor_comprehensive_test <logfile> <test_number>" << std::endl;
    std::cout << "  Test numbers: 1-4 (servo), 5-10 (DC), 11-12 (combined)" << std::endl;
    
    std::cout << "\n" << std::string(70, '=') << std::endl;
    std::cout << "All tests completed successfully!" << std::endl;
    std::cout << std::string(70, '=') << "\n" << std::endl;
    
    return 0;
}
