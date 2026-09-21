#ifndef SAMPLE_CONSOLE_H
#define SAMPLE_CONSOLE_H

#include <stdint.h>

namespace sample_console {

static const uint16_t BUFFER_SIZE = 96;
static const unsigned int UART_BAUD = 57600;
static const unsigned int UART_UBRR = 16000000UL / 16 / UART_BAUD - 1;

void initialize();
void readLine(char *buffer, uint16_t buffer_size);
bool isCommand(const char *line, const char *name);
bool getArgument(const char *line, const char *name, int16_t &value);
void printHeader(const char *title, const char *commands);
void printUnknownCommand();
void printValue(const char *label, int16_t value);

}

#endif
