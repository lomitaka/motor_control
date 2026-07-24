cmake_minimum_required(VERSION 3.12)
project(AVR_Simulator CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include cesty
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../src/internals)

# Zdrojové soubory simulátoru
set(SIMULATOR_SOURCES
    avr_simulator.cpp
)

# Zdrojové soubory vašeho kódu (bez arduino.cpp - použijeme mock)
set(MOTOR_CONTROL_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/timer_control.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/servo_control.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/dc_control.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/ramp.cpp
)

# Vytvoření knihovny simulátoru
add_library(avr_simulator STATIC ${SIMULATOR_SOURCES})

# Vytvoření testovacího programu
add_executable(motor_simulator 
    test_simulation.cpp
    ${MOTOR_CONTROL_SOURCES}
)

target_link_libraries(motor_simulator avr_simulator)

# Compiler flags pro simulaci
target_compile_definitions(motor_simulator PRIVATE 
    SIMULATION_MODE=1
    F_CPU=16000000UL
)

# Nahradit AVR includes simulovanými
target_compile_options(motor_simulator PRIVATE 
    -DAVR_MOCK
    -Wno-unused-variable
)

# Vytvoření komplexního testovacího programu
add_executable(motor_comprehensive_test
    test_comprehensive.cpp
    ${MOTOR_CONTROL_SOURCES}
)

target_link_libraries(motor_comprehensive_test avr_simulator)

target_compile_definitions(motor_comprehensive_test PRIVATE 
    SIMULATION_MODE=1
    F_CPU=16000000UL
)

target_compile_options(motor_comprehensive_test PRIVATE 
    -DAVR_MOCK
    -Wno-unused-variable
)

# Vytvoření testu různých scénářů motorů
add_executable(motor_scenario_test
    test_motor_scenarios.cpp
    ${MOTOR_CONTROL_SOURCES}
)

target_link_libraries(motor_scenario_test avr_simulator)

target_compile_definitions(motor_scenario_test PRIVATE 
    SIMULATION_MODE=1
    F_CPU=16000000UL
)

target_compile_options(motor_scenario_test PRIVATE 
    -DAVR_MOCK
    -Wno-unused-variable
)

# Build targets
add_custom_target(run_simulation
    COMMAND motor_simulator
    DEPENDS motor_simulator
    COMMENT "Running motor control simulation"
)

add_custom_target(run_comprehensive_test
    COMMAND motor_comprehensive_test
    DEPENDS motor_comprehensive_test
    COMMENT "Running comprehensive motor test suite"
)

add_custom_target(run_scenario_test
    COMMAND motor_scenario_test
    DEPENDS motor_scenario_test
    COMMENT "Running motor scenario tests (servo, DC, stepper)"
)

message(STATUS "AVR Simulator configured")
message(STATUS "Build with: cmake --build .")
message(STATUS "Run with: ./motor_simulator")
