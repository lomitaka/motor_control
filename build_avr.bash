#!/bin/bash
cd build
cmake -DTARGET_AVR=ON ..
make
