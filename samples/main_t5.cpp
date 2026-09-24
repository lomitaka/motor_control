#include "motor_control/dc_control.h"
#include "motor_control/dc_control_hbridge.h"
#include "sample_console.h"
#include "arduino.h"
#include <avr/interrupt.h>
#include "usart.h"

namespace {

DCControl motor1(13);
DCControlHBridge motor2(12,11,10);
char command[sample_console::BUFFER_SIZE];

void help() {
    sample_console::printHeader(
        "DC driver test",
        "1/2 <value> | i1/i2 <value> | stop | demo | help\r\n"
        "value range: 0..1000");
}

void demo() {
    USART_WRITE_S("DC demo: forward\r\n");
    motor1.setTarget(500);
    delay(2000);
    USART_WRITE_S("DC demo: reverse\r\n");
    motor1.setTarget(1000);
    delay(2000);
    motor1.setTarget(0);
    USART_WRITE_S("DC demo: stopped\r\n");
}

void processCommand(const char *line) {
    int16_t value;
    if (sample_console::isCommand(line, "help")) {
        help();
    } else if (sample_console::isCommand(line, "demo")) {
        demo();
    } else if (sample_console::isCommand(line, "stop")) {
        motor1.setImmediate(0);
        motor2.setImmediate(0);
        USART_WRITE_S("DC stopped\r\n");
    } else if (sample_console::getArgument(line, "1", value)) {
        if (value < 0) {value = 0; USART_WRITE_S("DC negative not allowed\r\n");}
        motor1.setTarget(value);
        sample_console::printValue("DC target1: ", value);
    } else if (sample_console::getArgument(line, "2", value)) {
        motor2.setTarget(value);
        sample_console::printValue("DC target2: ", value);
    } else if (sample_console::getArgument(line, "i1", value)) {
        motor1.setImmediate(value);
        sample_console::printValue("DC immediate1: ", value);
    } else if (sample_console::getArgument(line, "i2", value)) {
        motor2.setImmediate(value);
        sample_console::printValue("DC immediate2: ", value);
    } else {
        sample_console::printUnknownCommand();
    }
}

}

int main() {
    sample_console::initialize();
    sei();
    help();

    while (true) {
        sample_console::readLine(command, sample_console::BUFFER_SIZE);
        processCommand(command);
    }
}
