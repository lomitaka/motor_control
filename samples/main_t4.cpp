#include "motor_control/stepper_positioning.h"
#include "sample_console.h"
#include "arduino.h"
#include <avr/interrupt.h>
#include "usart.h"

namespace {

StepperPositioning motor;
char command[sample_console::BUFFER_SIZE];

void help() {
    sample_console::printHeader(
        "Positioning stepper test",
        "speed <steps/s> | accel <1..5> | move <steps> | add <steps> | immediate <steps> | stop | demo | help");
}

void demo() {
    USART_WRITE_S("Positioning demo: +400 steps\r\n");
    motor.setTargetTicks(400);
    while (motor.isMoving()) {
    }
    USART_WRITE_S("Positioning demo: -400 steps\r\n");
    motor.setTargetTicks(-400);
    while (motor.isMoving()) {
    }
    USART_WRITE_S("Positioning demo: complete\r\n");
}

void processCommand(const char *line) {
    int16_t value;
    if (sample_console::isCommand(line, "help")) {
        help();
    } else if (sample_console::isCommand(line, "demo")) {
        demo();
    } else if (sample_console::isCommand(line, "stop")) {
        motor.setImmediateTicks(0);
        USART_WRITE_S("Positioning stopped\r\n");
    } else if (sample_console::getArgument(line, "speed", value) && value >= 0) {
        motor.setSpeed(static_cast<uint16_t>(value));
        sample_console::printValue("Positioning speed: ", value);
    } else if (sample_console::getArgument(line, "accel", value) && value >= 1 && value <= 5) {
        motor.setAcceleration(static_cast<uint8_t>(value));
        sample_console::printValue("Positioning acceleration: ", value);
    } else if (sample_console::getArgument(line, "move", value)) {
        motor.setTargetTicks(value);
        sample_console::printValue("Move: ", value);
    } else if (sample_console::getArgument(line, "add", value)) {
        motor.addTargetTicks(value);
        sample_console::printValue("Added steps: ", value);
    } else if (sample_console::getArgument(line, "immediate", value)) {
        motor.setImmediateTicks(value);
        sample_console::printValue("Immediate steps: ", value);
    } else if (sample_console::isCommand(line, "status")) {
        USART_WRITE_S(motor.isMoving() ? "Moving\r\n" : "Stopped\r\n");
    } else {
        sample_console::printUnknownCommand();
    }
}

}

int main() {
    motor.init(2, 3);
    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    sample_console::initialize();
    sei();
    motor.setAcceleration(3);
    motor.setSpeed(400);
    help();

    while (true) {
        sample_console::readLine(command, sample_console::BUFFER_SIZE);
        processCommand(command);
    }
}
