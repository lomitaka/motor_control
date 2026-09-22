# Motor Control Library for AVR

C++ library for controlling DC motors, H-bridges, servos, and stepper motors on an ATmega328P-class AVR board. The default configuration targets an Arduino Uno-compatible board at 16 MHz.

## Requirements

On Debian or Ubuntu, install the AVR toolchain, uploader, and CMake:

```bash
sudo apt install cmake make arduino-core-avr avr-libc avrdude binutils-avr gcc-avr
```

The project uses these AVR packages:

- `arduino-core-avr`
- `avr-libc`
- `avrdude`
- `binutils-avr`
- `gcc-avr`

A native C++ compiler is also required for the simulator, for example `g++`.

## Build the AVR Library

The following build creates `build/libmotor_control_lib.a`, which is the path used by the sample projects:

```bash
mkdir -p build
cd build
cmake .. -DTARGET_AVR=ON
make motor_control_lib
```

Alternatively, build the complete default AVR target:

```bash
./build_avr.bash
```

`./build_library.bash` creates a library-only build in `build_lib/libmotor_control_lib.a`. If building a sample with the current sample CMake configuration, use the first command above or update its library path accordingly.

## Use in an AVR Program

Add the public headers from `include/` to the compiler include path and link against `libmotor_control_lib.a`. Initialize GPIO outputs, create driver objects, configure them, and call `sei()` only after application initialization is complete.

```cpp
#include "motor_control/dc_control.h"
#include <avr/interrupt.h>

DCControl motor(5);

int main() {
    pinMode(5, OUTPUT);
    motor.setAcceleration(500);
    sei();
    motor.setTarget(600);

    while (true) {
    }
}
```

Timer1 is shared by the drivers. Do not combine `StepperContinuous` with DC or servo drivers: it uses a different Timer1 configuration.

## Drivers

### DCControl

`DCControl` generates PWM on one pin. It accepts a target power from `0` to `1000`.

```cpp
#include "motor_control/dc_control.h"

DCControl motor(5);
motor.setAcceleration(500);  // 25..5000, percent per 100 ms
motor.setTarget(700);        // Smooth ramp to 70 percent PWM
motor.setImmediate(0);       // Stop immediately
```

### DCControlHBridge

`DCControlHBridge` drives a PWM pin and two direction pins. Its target range is `-1000..1000`; the sign selects direction. During a direction change, the driver ramps to zero before changing the direction pins.

```cpp
#include "motor_control/dc_control_hbridge.h"

DCControlHBridge motor(5, 4, 7);
motor.setAcceleration(500);
motor.setTarget(600);
motor.setTarget(-600);
motor.setImmediate(0);
```

### ServoControl

`ServoControl` produces servo control pulses on an Arduino pin. Values are currently passed as `int16_t` targets.

```cpp
#include "motor_control/servo_control.h"

ServoControl servo(9);
servo.setTarget(0);
servo.setTarget(700);
```

### StepperContinuous

`StepperContinuous` controls a STEP/DIR driver at a requested continuous speed.

```cpp
#include "motor_control/stepper_continuous.h"

StepperContinuous motor(2, 3);
motor.setAcceleration(500);
motor.setTargetSpeed(1000);
motor.setTargetSpeed(-500);
motor.setImmediateSpeed(0);
```

The speed range is clamped to `-5000..5000` steps/s.

### StepperPositioning

`StepperPositioning` makes a finite movement in steps with acceleration and deceleration.

```cpp
#include "motor_control/stepper_positioning.h"

StepperPositioning motor(2, 3);
motor.setSpeed(800);
motor.setAcceleration(3);
motor.setTargetTicks(400);
```

Useful methods include `addTargetTicks()`, `setImmediateTicks()`, and `isMoving()`.

## Samples

The `samples/` directory contains UART-controlled AVR examples. They use 57600 baud, 8 data bits, no parity, and 1 stop bit.

| `SAMPLE_NO` | Source | Driver |
| --- | --- | --- |
| `0` | `main_t0.cpp` | One-pin DC PWM |
| `1` | `main_t1.cpp` | DC H-bridge |
| `2` | `main_t2.cpp` | Continuous stepper |
| `3` | `main_t3.cpp` | Positioning stepper |
| `4` | `main_t4.cpp` | Positioning stepper debug test |
| `5` | `main_t5.cpp` | Additional positioning-stepper sample |

First build the library in the root `build/` directory, then configure a sample. For example, to build the H-bridge sample:

```bash
cmake -S . -B build -DTARGET_AVR=ON
cmake --build build --target motor_control_lib

cmake -S samples -B samples/build -DTARGET_AVR=ON -DSAMPLE_NO=1
cmake --build samples/build --target line_follower.elf
```

The generated files are `samples/build/line_follower.elf` and `samples/build/line_follower.hex`.

To upload using the configured `/dev/ttyACM0` port:

```bash
cmake --build samples/build --target upload
```

Use `uploadUSB` instead when the board is available as `/dev/ttyUSB0`.

The H-bridge sample accepts commands such as:

```text
set 500
set -500
accel 500
stop
demo
status
help
```

## Simulator

The simulator executes the driver logic on the host and records pin changes and calculated motor load.

```bash
./build_simulator.bash
cd simulator/build
./dc_test
```

Other simulator executables include:

```bash
./motor_simulator
./motor_comprehensive_test
./motor_scenario_test
./stepper_c_test
./stepper_p_test
```

CMake custom targets can also run selected simulations:

```bash
cmake --build simulator/build --target run_dc_test
cmake --build simulator/build --target run_simulation
```

The simulator writes log files into the current working directory. For the DC test these include `dc_simulation.log` and `dc_simulation_load.log`.
