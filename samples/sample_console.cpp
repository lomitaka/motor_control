#include "sample_console.h"
#include "usart.h"
#include "arduino.h"
#include <avr/io.h>
#include <avr/interrupt.h>

namespace sample_console {

void initialize() {
    ADMUX = (ADMUX & 0x3F) | (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    USART_Init(UART_UBRR);
    sei();
}

void readLine(char *buffer, uint16_t buffer_size) {
    uint16_t index = 0;

    while (true) {
        int received = USART_Receive();
        if (received == '\n' || received == '\r') {
            buffer[index] = '\0';
            return;
        }
        if (index + 1 < buffer_size) {
            buffer[index++] = static_cast<char>(received);
        }
    }
}

bool isCommand(const char *line, const char *name) {
    uint16_t index = 0;
    while (name[index] != '\0') {
        if (line[index] != name[index]) {
            return false;
        }
        ++index;
    }
    return line[index] == '\0' || line[index] == ' ' || line[index] == '\t';
}

bool getArgument(const char *line, const char *name, int16_t &value) {
    if (!isCommand(line, name)) {
        return false;
    }

    uint16_t index = 0;
    while (name[index] != '\0') {
        ++index;
    }
    while (line[index] == ' ' || line[index] == '\t') {
        ++index;
    }
    if (line[index] == '\0') {
        return false;
    }

    int8_t sign = 1;
    if (line[index] == '-') {
        sign = -1;
        ++index;
    } else if (line[index] == '+') {
        ++index;
    }

    if (line[index] < '0' || line[index] > '9') {
        return false;
    }

    int32_t result = 0;
    while (line[index] >= '0' && line[index] <= '9') {
        result = result * 10 + line[index] - '0';
        ++index;
    }
    if (line[index] != '\0' && line[index] != ' ' && line[index] != '\t') {
        return false;
    }

    result *= sign;
    if (result < -32768 || result > 32767) {
        return false;
    }
    value = static_cast<int16_t>(result);
    return true;
}

void printHeader(const char *title, const char *commands) {
    USART_WRITE_S("\r\n");
    USART_WRITE_S(title);
    USART_WRITE_S("\r\n");
    USART_WRITE_S(commands);
    USART_WRITE_S("\r\n");
}

void printUnknownCommand() {
    USART_WRITE_S("Unknown command. Type help.\r\n");
}

void printValue(const char *label, int16_t value) {
    USART_WRITE_S(label);
    USART_WRITE_LLONG(value);
    USART_WRITE_S("\r\n");
}

}
