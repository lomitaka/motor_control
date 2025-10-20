cmake_minimum_required(VERSION 3.12)
project(test_lib C CXX)

# Nastavte platformu a procesor
set(MCU atmega328p)
set(F_CPU 16000000UL)  # Nastavte frekvenci procesoru (zde 16 MHz pro Arduino Uno)

option(TARGET_AVR "Compile for AVR microcontroller" ON)

include_directories(${CMAKE_SOURCE_DIR})

# Nastavte zdrojové soubory
set(SOURCES

    src/arduino.cpp  
    src/usart.cpp
    src/CircularLogger.cpp
    src/Sketch.cpp   
    src/timer_control.cpp
    #src/motor.cpp
    src/ramp.cpp


)


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
    # Vytvoření spustitelného souboru pro AVR
    add_executable(test_lib.elf ${SOURCES})

    # Přidejte pravidla pro generování HEX souboru
    add_custom_command(TARGET test_lib.elf POST_BUILD
        COMMAND avr-objcopy -O ihex -R .eeprom test_lib.elf test_lib.hex
        COMMENT "Generating HEX file from ELF"
    )

    # Možnost nahrát do Arduina pomocí `avrdude`
    add_custom_target(upload
        #COMMAND avrdude -v -patmega328p -carduino -P/dev/ttyUSB0 -b115200 -D -Uflash:w:test_lib.hex:i
        COMMAND avrdude -v -patmega328p -carduino -P/dev/ttyACM0 -b115200 -D -Uflash:w:test_lib.hex:i
        DEPENDS test_lib.elf
        COMMENT "Uploading to Arduino"
    )

    add_custom_target(uploadUSB
        #COMMAND avrdude -v -patmega328p -carduino -P/dev/ttyUSB0 -b115200 -D -Uflash:w:test_lib.hex:i
        COMMAND avrdude -v -patmega328p -carduino -P/dev/ttyUSB0 -b115200 -D -Uflash:w:test_lib.hex:i
        DEPENDS test_lib.elf
        COMMENT "Uploading to Arduino"
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


