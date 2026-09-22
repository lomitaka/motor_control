cmake_minimum_required(VERSION 3.12)
project(motor_control C CXX)

# Nastavte platformu a procesor
set(MCU atmega328p)
set(F_CPU 16000000UL)  # Nastavte frekvenci procesoru (zde 16 MHz pro Arduino Uno)

option(TARGET_AVR "Compile for AVR microcontroller" ON)
option(BUILD_LIBRARY_ONLY "Build only the library, not the executable" OFF)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/include)

# Nastavte zdrojové soubory pro knihovnu
set(LIBRARY_SOURCES
    src/timer_control.cpp
    src/dc_control_common.cpp
    src/dc_control.cpp
    src/dc_control_hbridge.cpp
    src/servo_control.cpp
    src/arduino.cpp
    src/stepp_control.cpp
    src/stepp_timer_control.cpp
    src/stepc_control.cpp
    src/stepc_timer_control.cpp
)


if(TARGET_AVR)
    # Nastavení pro AVR
    set(CMAKE_SYSTEM_NAME Generic)
    set(CMAKE_C_COMPILER avr-gcc)
    set(CMAKE_CXX_COMPILER avr-g++)
    set(CMAKE_C_FLAGS "-Os -DF_CPU=${F_CPU} -mmcu=${MCU} -Wall -Wextra")
    set(CMAKE_CXX_FLAGS "-Os -DF_CPU=${F_CPU} -mmcu=${MCU} -Wall -Wextra -std=c++11")
    set(CMAKE_EXE_LINKER_FLAGS "-Wl,-gc-sections")
else()
    # Nastavení pro x86 (standardní GCC)
    set(CMAKE_C_COMPILER gcc)
    set(CMAKE_CXX_COMPILER g++)
    set(CMAKE_CXX_STANDARD 11)
    set(CMAKE_CXX_FLAGS "-Wall -Wextra -g")
    set(CMAKE_C_FLAGS "-Wall -Wextra -g")
endif()


# Přidejte spustitelný soubor
if(TARGET_AVR)
    # Vytvoření statické knihovny pro AVR
    add_library(motor_control_lib STATIC ${LIBRARY_SOURCES})
      # Nastavení include directories pro knihovnu
    target_include_directories(motor_control_lib PUBLIC 
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}
    )

    # Instalace knihovny a hlavičkových souborů
    install(TARGETS motor_control_lib 
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
    )
    
    install(DIRECTORY include/ 
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
    )
    
    install(DIRECTORY src/internals/
        DESTINATION include/internals
        FILES_MATCHING PATTERN "*.h"
    )

else()
# -------------------------------------------------
# Přidání cíle pro testovací binárku na x86 platformě
# -------------------------------------------------
# Nastavení pro standardní platformu (x86)
#set(X86_TEST_SOURCES
#    logic.cpp
#    test/logic_test.cpp  # Testovací kód
#    test/usart_test.cpp
#    CircularLogger.cpp
#    getRotationDelta.cpp
#)

#add_executable(logic_test ${X86_TEST_SOURCES})
#target_compile_options(logic_test PRIVATE -Wall -Wextra -std=c++11)

    
endif()


