#!/bin/bash

# Skript pro sestavení a spuštění AVR simulátoru

echo "Building AVR Motor Control Simulator..."

# Vytvoření build adresáře
mkdir -p simulator/build
cd simulator/build

# Konfigurace CMake
cmake .. -DCMAKE_BUILD_TYPE=Release

# Sestavení
make -j$(nproc)

echo ""
echo "Build completed!"
echo "Run simulation with: ./simulator/build/motor_simulator"
echo "Or use: cd simulator/build && ./motor_simulator"
