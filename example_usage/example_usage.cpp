// Příklad použití motor_control knihovny
#include <avr/io.h>
#include <avr/interrupt.h>

// Include z vaší knihovny
#include "motor_control/dc_control.h"
#include "motor_control/servo_control.h"
#include "internals/timer_control.h"

int main() {
    // Inicializace timerů
    TimerControl::setup_Timers();
    
    // Nastavení DC motoru na index 0, port B, pin 2
    DCControl::setDCMotorPortPin(0, 2, 2);  // PORTB, pin 2
    
    // Nastavení hodnoty motoru (rozsah -1000 až 1000)
    DCControl::setDCMotorValue(0, 500);  // 50% dopředu
    
    // Nekonečná smyčka
    while(1) {
        // Váš hlavní kód zde
    }
    
    return 0;
}
