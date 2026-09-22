#include "motor_control/stepper_continuous.h"
#include "sample_console.h"
#include "arduino.h"
#include <avr/interrupt.h>
#include "usart.h"

namespace {

StepperContinuous motor;
char command[sample_console::BUFFER_SIZE];

void help() {
    sample_console::printHeader(
        "Continuous stepper test",
        "speed <steps/s> | immediate <steps/s> | accel <steps/s2> | stop | demo | help");
}

void demo() {
    USART_WRITE_S("Continuous stepper demo: forward\r\n");
    motor.setTargetSpeed(400);
    delay(3000);
    USART_WRITE_S("Continuous stepper demo: reverse\r\n");
    motor.setTargetSpeed(-400);
    delay(3000);
    motor.setTargetSpeed(0);
    USART_WRITE_S("Continuous stepper demo: stopped\r\n");
}

void processCommand(const char *line) {
    int16_t value;
    if (sample_console::isCommand(line, "help")) {
        help();
    } else if (sample_console::isCommand(line, "demo")) {
        demo();
    } else if (sample_console::isCommand(line, "stop")) {
        motor.setImmediateSpeed(0);
        USART_WRITE_S("Stepper stopped\r\n");
    } else if (sample_console::getArgument(line, "speed", value)) {
        motor.setTargetSpeed(value);
        sample_console::printValue("Target speed: ", value);
    } else if (sample_console::getArgument(line, "immediate", value)) {
        motor.setImmediateSpeed(value);
        sample_console::printValue("Immediate speed: ", value);
    } else if (sample_console::getArgument(line, "accel", value) && value >= 0) {
        motor.setAcceleration(static_cast<uint16_t>(value));
        sample_console::printValue("Acceleration: ", value);
    } else if (sample_console::isCommand(line, "status")) {
        sample_console::printValue("Current speed: ", motor.getCurrentSpeed());
        sample_console::printValue("Target speed: ", motor.getTargetSpeed());
    } else {
        sample_console::printUnknownCommand();
    }
}

}

int main() {
    motor.init(2, 3);
    sample_console::initialize();
    sei();
    motor.setAcceleration(100);
    help();

    while (true) {
        sample_console::readLine(command, sample_console::BUFFER_SIZE);
        processCommand(command);
    }
}
