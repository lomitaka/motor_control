#!/bin/bash

# Skript pro instalaci knihovny do systému

INSTALL_PREFIX=${1:-"/usr/local"}

echo "Installing motor control library to $INSTALL_PREFIX..."

# Sestavení knihovny nejdříve
./build_library.bash

cd build_lib

# Instalace knihovny a hlavičkových souborů
sudo make install

echo "Library installed to:"
echo "  Headers: $INSTALL_PREFIX/include/"
echo "  Library: $INSTALL_PREFIX/lib/libmotor_control_lib.a"
