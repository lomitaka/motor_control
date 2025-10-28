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
    src/dc_control.cpp
    src/servo_control.cpp
    src/ramp.cpp
    src/arduino.cpp
)

# Zdrojové soubory pro executable (hlavní program)
set(EXECUTABLE_SOURCES
    test/Sketch.cpp
    test/usart.cpp
    test/arduino.cpp
)

# This may be handy when will build library (but does not work as chat gpt suggested that)
#target_include_directories(motor_control
#    PUBLIC
#        ${CMAKE_CURRENT_SOURCE_DIR}/include
#    PRIVATE
#        ${CMAKE_CURRENT_SOURCE_DIR}/src/internal
#)


# Nastavení kompilátoru na AVR GCC
#set(CMAKE_SYSTEM_NAME Generic)
#set(CMAKE_C_COMPILER avr-gcc)
#set(CMAKE_CXX_COMPILER avr-g++)
#make it c++ 11
#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Nastavení pro AVR GCC
#set(CMAKE_C_FLAGS "-Os -DF_CPU=${F_CPU} -mmcu=${MCU}")
#set(CMAKE_CXX_FLAGS "-Os -DF_CPU=${F_CPU} -mmcu=${MCU}")
#set(CMAKE_EXE_LINKER_FLAGS "-Wl,-gc-sections")


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
    
    # Vytvoření spustitelného souboru pouze pokud není BUILD_LIBRARY_ONLY
    if(NOT BUILD_LIBRARY_ONLY)
        # Vytvoření spustitelného souboru pro AVR
        add_executable(motor_control.elf ${EXECUTABLE_SOURCES})
          # Linkování knihovny k executable
        target_link_libraries(motor_control.elf motor_control_lib)

        # Přidejte pravidla pro generování HEX souboru
        add_custom_command(TARGET motor_control.elf POST_BUILD
            COMMAND avr-objcopy -O ihex -R .eeprom motor_control.elf motor_control.hex
            COMMENT "Generating HEX file from ELF"
        )

        # Možnost nahrát do Arduina pomocí `avrdude`
        add_custom_target(upload
            #COMMAND avrdude -v -patmega328p -carduino -P/dev/ttyUSB0 -b115200 -D -Uflash:w:motor_control.hex:i
            COMMAND avrdude -v -patmega328p -carduino -P/dev/ttyACM0 -b115200 -D -Uflash:w:motor_control.hex:i
            DEPENDS motor_control.elf
            COMMENT "Uploading to Arduino"
        )

        add_custom_target(uploadUSB
            #COMMAND avrdude -v -patmega328p -carduino -P/dev/ttyUSB0 -b115200 -D -Uflash:w:motor_control.hex:i
            COMMAND avrdude -v -patmega328p -carduino -P/dev/ttyUSB0 -b115200 -D -Uflash:w:motor_control.hex:i
            DEPENDS motor_control.elf            
            COMMENT "Uploading to Arduino"
        )
    endif()

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


