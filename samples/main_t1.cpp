#include "motor_control/dc_control_hbridge.h"
#include "sample_console.h"
#include "arduino.h"
#include <avr/interrupt.h>
#include "usart.h"

namespace {

constexpr uint8_t PWM_PIN = 5;
constexpr uint8_t DIRECTION_A_PIN = 4;
constexpr uint8_t DIRECTION_B_PIN = 7;

DCControlHBridge motor(PWM_PIN, DIRECTION_A_PIN, DIRECTION_B_PIN);
char command[sample_console::BUFFER_SIZE];
uint16_t acceleration = 500;

/* Note: tested with monseter shield 
    VNH3SP30 Dual Monster Motor Driver Shield
    Arduino:   Driver:   Meaning 
    5 -        5 -       pwm
    4          8         dir A
    7          7         dir B
*/

void help() {
    sample_console::printHeader(
        "H-bridge DC driver test",
        "set <value> | immediate <value> | accel <25..5000> | stop | demo | status | help\r\n"
        "value range: -1000..1000, acceleration: percent per 100 ms");
}

void demo() {
    USART_WRITE_S("H-bridge demo: forward\r\n");
    motor.setTarget(500);
    delay(2000);

    USART_WRITE_S("H-bridge demo: reverse\r\n");
    motor.setTarget(-500);
    delay(4000);

    motor.setTarget(0);
    USART_WRITE_S("H-bridge demo: stopped\r\n");
}

void processCommand(const char *line) {
    int16_t value;

    if (sample_console::isCommand(line, "help")) {
        help();
    } else if (sample_console::isCommand(line, "demo")) {
        demo();
    } else if (sample_console::isCommand(line, "stop")) {
        motor.setImmediate(0);
        USART_WRITE_S("H-bridge stopped\r\n");
    } else if (sample_console::isCommand(line, "status")) {
        sample_console::printValue("Acceleration: ", acceleration);
    } else if (sample_console::getArgument(line, "set", value)) {
        motor.setTarget(value);
        sample_console::printValue("H-bridge target: ", value);
    } else if (sample_console::getArgument(line, "immediate", value)) {
        motor.setImmediate(value);
        sample_console::printValue("H-bridge immediate: ", value);
    } else if (sample_console::getArgument(line, "accel", value)) {
        if (value < 25) {
            value = 25;
        } else if (value > 5000) {
            value = 5000;
        }
        acceleration = static_cast<uint16_t>(value);
        motor.setAcceleration(acceleration);
        sample_console::printValue("Acceleration set to: ", acceleration);
    } else {
        sample_console::printUnknownCommand();
    }
}

}

int main() {
    sample_console::initialize();
    motor.setAcceleration(acceleration);
    sei();
    help();

    while (true) {
        sample_console::readLine(command, sample_console::BUFFER_SIZE);
        processCommand(command);
    }
}
