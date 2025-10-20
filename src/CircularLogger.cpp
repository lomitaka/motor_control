#include "headers/usart.h"

#define LOG_SIZE 2  // Size of the circular log buffer

char logBuffer[LOG_SIZE];
volatile size_t logIndex = 0;  // Points to the current position in the buffer
volatile size_t logCount = 0;  // Number of valid characters in the buffer

void logChar(char c) {
	logBuffer[logIndex] = c;
	logIndex = (logIndex + 1) % LOG_SIZE;

	// If the buffer is full, overwrite the oldest character
	if (logCount < LOG_SIZE) {
		logCount++;
	}
}

void flushLog() {
	// Flushes the log by printing all valid characters
	size_t startIdx = (logIndex + LOG_SIZE - logCount) % LOG_SIZE;

	for (size_t i = 0; i < logCount; i++) {
		size_t idx = (startIdx + i) % LOG_SIZE;
		USART_Transmit(logBuffer[idx]);
	}

	// Reset the buffer after flushing
	logCount = 0;
	logIndex = 0;
}

void buttonPressHandler() {
	// Call this function when the button is pressed
	flushLog();
}