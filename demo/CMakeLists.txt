cmake_minimum_required(VERSION 3.12)
project(motor_driver_sample C CXX)

# Nastavte platformu a procesor
set(MCU atmega328p)
set(F_CPU 16000000UL)  # Nastavte frekvenci procesoru (zde 16 MHz pro Arduino Uno)
set(SAMPLE_NO "4" CACHE STRING "Sample to build")

option(TARGET_AVR "Compile for AVR microcontroller" ON)

# Nastavení cest k include adresářům
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)



# Nastavte zdrojové soubory
set(SOURCES
    #main.cpp
    arduino.cpp
    usart.cpp
    #../include/motor_control/stepper_positioning.h
)

if("${SAMPLE_NO}" STREQUAL "DC")
    list(APPEND SOURCES main_dc.cpp)
elseif("${SAMPLE_NO}" STREQUAL "1")
    list(APPEND SOURCES main_t1.cpp)
elseif("${SAMPLE_NO}" STREQUAL "2")
    list(APPEND SOURCES main_t2.cpp)
elseif("${SAMPLE_NO}" STREQUAL "3")
    list(APPEND SOURCES main_t3.cpp)
elseif("${SAMPLE_NO}" STREQUAL "4")
    list(APPEND SOURCES main_t4.cpp)
else()
    message(FATAL_ERROR "Unknown test: ${TEST}")
endif()


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


if(TARGET_AVR)
    # Vytvoření spustitelného souboru pro AVR
    add_executable(demo.elf ${SOURCES})
    
    # Linkování motor_control knihovny
    target_link_libraries(demo.elf ${CMAKE_CURRENT_SOURCE_DIR}/../build/libmotor_control_atmega328p_dcservo_timer1.a)

    # Přidejte pravidla pro generování HEX souboru
    add_custom_command(TARGET demo.elf POST_BUILD
        COMMAND avr-objcopy -O ihex -R .eeprom demo.elf demo.hex
        COMMENT "Generating HEX file from ELF"
    )    # Možnost nahrát do Arduina pomocí `avrdude`
    add_custom_target(upload
        #COMMAND avrdude -v -patmega328p -carduino -P/dev/ttyUSB0 -b115200 -D -Uflash:w:demo.hex:i
        COMMAND avrdude -v -patmega328p -carduino -P/dev/ttyACM0 -b115200 -D -Uflash:w:demo.hex:i
        DEPENDS demo.elf
        COMMENT "Uploading to Arduino"
    )

    add_custom_target(uploadUSB
        #COMMAND avrdude -v -patmega328p -carduino -P/dev/ttyUSB0 -b115200 -D -Uflash:w:demo.hex:i
        COMMAND avrdude -v -patmega328p -carduino -P/dev/ttyUSB0 -b115200 -D -Uflash:w:demo.hex:i
        DEPENDS demo.elf
        COMMENT "Uploading to Arduino"
    )

endif()


