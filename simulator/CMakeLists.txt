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
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/dc_control_common.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/servo_control.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/dc_control.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/dc_control_hbridge.cpp
)

# Zdrojové soubory pro stepper motor control
set(STEPPER_C_CONTROL_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/stepc_control.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/stepc_timer_control.cpp
)

# Zdrojové soubory pro stepper motor control
set(STEPPER_P_CONTROL_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/stepp_control.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/stepp_timer_control.cpp
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

add_executable(dc_test
    test_dc.cpp
    ${MOTOR_CONTROL_SOURCES}
)

target_link_libraries(dc_test avr_simulator)

target_compile_definitions(motor_scenario_test PRIVATE 
    SIMULATION_MODE=1
    F_CPU=16000000UL
)

target_compile_definitions(dc_test PRIVATE
    SIMULATION_MODE=1
    F_CPU=16000000UL
)

target_compile_options(motor_scenario_test PRIVATE 
    -DAVR_MOCK
    -Wno-unused-variable
)

target_compile_options(dc_test PRIVATE
    -DAVR_MOCK
    -Wno-unused-variable
)

# Vytvoření testu pro stepper motory
add_executable(stepper_c_test
    ${CMAKE_CURRENT_SOURCE_DIR}/test_stepper.cpp
    ${STEPPER_C_CONTROL_SOURCES}
)

# Vytvoření testu pro stepper motory
add_executable(stepper_p_test
    ${CMAKE_CURRENT_SOURCE_DIR}/test_stepper_p.cpp
    ${STEPPER_P_CONTROL_SOURCES}
)

target_link_libraries(stepper_p_test avr_simulator)
target_link_libraries(stepper_c_test avr_simulator)

target_compile_definitions(stepper_c_test PRIVATE 
    SIMULATION_MODE=1
    F_CPU=16000000UL
)
target_compile_definitions(stepper_p_test PRIVATE 
    SIMULATION_MODE=1
    F_CPU=16000000UL
)

target_compile_options(stepper_c_test PRIVATE 
    -DAVR_MOCK
    -Wno-unused-variable
)

target_compile_options(stepper_p_test PRIVATE 
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

add_custom_target(run_dc_test
    COMMAND dc_test
    DEPENDS dc_test
    COMMENT "Running the minimal DC motor simulation test"
)

add_custom_target(run_p_stepper_test
    COMMAND stepper_p_test
    DEPENDS stepper_p_test
    COMMENT "Running stepper motor test"
)

message(STATUS "AVR Simulator configured")
message(STATUS "Build with: cmake --build .")
message(STATUS "Run with: ./motor_simulator")
