#!/bin/bash

# Skript pro sestavení pouze knihovny pro AVR

echo "Building motor control library for AVR..."

# Vytvoření build adresáře pro knihovnu
mkdir -p build_lib
cd build_lib

# Konfigurace CMake pro AVR s knihovnou
cmake .. -DTARGET_AVR=ON -DBUILD_LIBRARY_ONLY=ON

# Sestavení knihovny
make motor_control_lib

echo "Library build completed!"
echo "Library file: build_lib/libmotor_control_lib.a"
