#include "motor_control/dc_control.h"
#include "sample_console.h"
#include "arduino.h"
#include <avr/interrupt.h>
#include "usart.h"

namespace {

DCControl motor(5);
char command[sample_console::BUFFER_SIZE];

void help() {
    sample_console::printHeader(
        "DC driver test",
        "set <value> | immediate <value> | stop | demo | help\r\n"
        "value range: 0..1000");
}

void demo() {
    USART_WRITE_S("DC demo: forward\r\n");
    motor.setTarget(500);
    delay(2000);
    USART_WRITE_S("DC demo: reverse\r\n");
    motor.setTarget(1000);
    delay(2000);
    motor.setTarget(0);
    USART_WRITE_S("DC demo: stopped\r\n");
}

void processCommand(const char *line) {
    int16_t value;
    if (sample_console::isCommand(line, "help")) {
        help();
    } else if (sample_console::isCommand(line, "demo")) {
        demo();
    } else if (sample_console::isCommand(line, "stop")) {
        motor.setImmediate(0);
        USART_WRITE_S("DC stopped\r\n");
    } else if (sample_console::getArgument(line, "set", value)) {
        motor.setTarget(value);
        sample_console::printValue("DC target: ", value);
    } else if (sample_console::getArgument(line, "immediate", value)) {
        motor.setImmediate(value);
        sample_console::printValue("DC immediate: ", value);
    } else {
        sample_console::printUnknownCommand();
    }
}

}

int main() {
    pinMode(5, OUTPUT);
    sample_console::initialize();
    sei();
    help();

    while (true) {
        sample_console::readLine(command, sample_console::BUFFER_SIZE);
        processCommand(command);
    }
}
