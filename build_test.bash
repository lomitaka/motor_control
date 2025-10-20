#!/bin/bash
cd build_test
cmake -DTARGET_AVR=OFF ..
make 
