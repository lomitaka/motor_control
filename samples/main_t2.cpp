#include "motor_control/servo_control.h"
#include "sample_console.h"
#include "arduino.h"
#include <avr/interrupt.h>
#include "usart.h"

namespace {

ServoControl servo(9);
char command[sample_console::BUFFER_SIZE];

void help() {
    sample_console::printHeader(
        "Servo driver test",
        "set <value> | immediate <value> | stop | demo | help\r\n"
        "value range: -1000..1000");
}

void demo() {
    USART_WRITE_S("Servo demo: center -> left -> center -> right -> center\r\n");
    servo.setTarget(0);
    delay(1000);
    servo.setTarget(-700);
    delay(1500);
    servo.setTarget(0);
    delay(1000);
    servo.setTarget(700);
    delay(1500);
    servo.setTarget(0);
}

void processCommand(const char *line) {
    int16_t value;
    if (sample_console::isCommand(line, "help")) {
        help();
    } else if (sample_console::isCommand(line, "demo")) {
        demo();
    } else if (sample_console::isCommand(line, "stop")) {
        servo.setImmediate(0);
        USART_WRITE_S("Servo centered\r\n");
    } else if (sample_console::getArgument(line, "set", value)) {
        servo.setTarget(value);
        sample_console::printValue("Servo target: ", value);
    } else if (sample_console::getArgument(line, "immediate", value)) {
        servo.setImmediate(value);
        sample_console::printValue("Servo immediate: ", value);
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
