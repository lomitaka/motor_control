#!/bin/bash
# Build stepper motor test program for simulator

echo "Building stepper motor test..."

# Vytvoř build adresář pro test
mkdir -p test/build
cd test/build

# Compile simulator core
g++ -c ../../simulator/avr_simulator.cpp \
    -I../../simulator \
    -I../../ \
    -I../../include \
    -I../../src \
    -I../../src/internals \
    -DSIMULATION_MODE=1 \
    -DF_CPU=16000000UL \
    -std=c++11 \
    -o avr_simulator.o

# Compile stepper control
g++ -c ../../src/step_control.cpp \
    -I../../simulator \
    -I../../ \
    -I../../include \
    -I../../src \
    -I../../src/internals \
    -DSIMULATION_MODE=1 \
    -DF_CPU=16000000UL \
    -std=c++11 \
    -o step_control.o

# Compile stepper timer
g++ -c ../../src/step_timer_control.cpp \
    -I../../simulator \
    -I../../ \
    -I../../include \
    -I../../src \
    -I../../src/internals \
    -DSIMULATION_MODE=1 \
    -DF_CPU=16000000UL \
    -std=c++11 \
    -o step_timer_control.o

# Compile test program
g++ -c ../test_stepper.cpp \
    -I../../simulator \
    -I../../ \
    -I../../include \
    -I../../src \
    -I../../src/internals \
    -DSIMULATION_MODE=1 \
    -DF_CPU=16000000UL \
    -std=c++11 \
    -o test_stepper.o

# Link all together
g++ avr_simulator.o step_control.o step_timer_control.o test_stepper.o \
    -o stepper_test \
    -std=c++11

if [ $? -eq 0 ]; then
    echo "Build successful!"
    echo "Run with: ./test/build/stepper_test"
else
    echo "Build failed!"
    exit 1
fi
