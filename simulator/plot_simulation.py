#!/usr/bin/env python3
"""
AVR Motor Simulation Log Visualizer
Parsuje motor_simulation.log a vytváří graf PWM signálů
"""

import re
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict

def parse_log_file(filename):
    """
    Parsuje log soubor a vrací data pro každý pin.
    
    Returns:
        dict: {pin_name: [(time_us, state), ...]}
    """
    pin_data = defaultdict(list)
    
    # Regex pro parsování řádků: [    4096.000 us] PORTB.2 = HIGH  (TCNT1=0)
    pattern = r'\[\s*([\d.]+)\s*us\]\s+PORT([BCD])\.(\d+)\s+=\s+(HIGH|LOW)'
    
    with open(filename, 'r') as f:
        for line in f:
            match = re.search(pattern, line)
            if match:
                time_us = float(match.group(1))
                port = match.group(2)
                pin_num = match.group(3)
                state = match.group(4)
                
                # Název pinu (např. "PORTB.2")
                pin_name = f"PORT{port}.{pin_num}"
                
                # Převod HIGH/LOW na 1/0
                value = 1 if state == "HIGH" else 0
                
                pin_data[pin_name].append((time_us, value))
    
    return pin_data

def create_pwm_plot(pin_data, output_file='pwm_signals.png'):
    """
    Vytvoří graf PWM signálů pro všechny piny.
    
    Args:
        pin_data: Dictionary s daty pro každý pin
        output_file: Název výstupního souboru
    """
    if not pin_data:
        print("No data to plot!")
        return
    
    # Vytvoření figury
    fig, ax = plt.subplots(figsize=(14, 8))
    
    # Barvy pro jednotlivé piny
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', 
              '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
    
    # Offset pro každý pin (vertikální oddělení signálů)
    pin_offset = {}
    offset = 0
    
    # Seřazení pinů podle názvu
    sorted_pins = sorted(pin_data.keys())
    
    for idx, pin_name in enumerate(sorted_pins):
        pin_offset[pin_name] = offset
        data = pin_data[pin_name]
        
        if not data:
            continue
        
        # Převod na numpy arrays
        times = np.array([t for t, v in data])
        values = np.array([v for t, v in data])
        
        # Vytvoření "step" signálu pro PWM
        # Pro každou změnu vytvoříme dva body (horizontální a vertikální čáry)
        plot_times = []
        plot_values = []
        
        # První bod
        plot_times.append(times[0])
        plot_values.append(values[0] + offset)
        
        # Pro každou změnu stavu
        for i in range(1, len(times)):
            # Horizontální čára (zachování předchozího stavu)
            plot_times.append(times[i])
            plot_values.append(values[i-1] + offset)
            
            # Vertikální čára (změna stavu)
            plot_times.append(times[i])
            plot_values.append(values[i] + offset)
        
        # Poslední horizontální čára až do konce grafu
        if len(times) > 0:
            plot_times.append(times[-1] + 1000)  # +1ms za poslední událost
            plot_values.append(values[-1] + offset)
        
        # Vykreslení signálu
        color = colors[idx % len(colors)]
        ax.plot(plot_times, plot_values, linewidth=2, color=color, label=pin_name)
        
        # Přidání mřížky pro tento pin
        ax.axhline(y=offset + 0.5, color='gray', linestyle=':', linewidth=0.5, alpha=0.3)
        
        offset += 2  # Mezera mezi signály
    
    # Nastavení grafu
    ax.set_xlabel('Time (μs)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Signal Level', fontsize=12, fontweight='bold')
    ax.set_title('AVR Motor Control - PWM Signals', fontsize=14, fontweight='bold')
    ax.grid(True, axis='x', alpha=0.3)
    ax.legend(loc='upper right', fontsize=10)
    
    # Nastavení Y-osy - zobrazit jen názvy pinů
    y_ticks = [pin_offset[pin] + 0.5 for pin in sorted_pins]
    ax.set_yticks(y_ticks)
    ax.set_yticklabels(sorted_pins)
    
    # Nastavení limitů
    ax.set_ylim(-0.5, offset + 0.5)
    
    # Formátování X-osy
    ax.ticklabel_format(axis='x', style='plain')
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"✓ Graf uložen do: {output_file}")
    plt.show()

def print_statistics(pin_data):
    """Vypíše statistiky o signálech."""
    print("\n" + "="*60)
    print("SIGNAL STATISTICS")
    print("="*60)
    
    for pin_name in sorted(pin_data.keys()):
        data = pin_data[pin_name]
        if len(data) < 2:
            continue
        
        times = [t for t, v in data]
        values = [v for t, v in data]
        
        # Výpočet pulse width (doba HIGH stavu)
        high_times = []
        low_times = []
        
        for i in range(len(data) - 1):
            duration = times[i+1] - times[i]
            if values[i] == 1:
                high_times.append(duration)
            else:
                low_times.append(duration)
        
        if high_times:
            avg_high = np.mean(high_times)
            print(f"\n{pin_name}:")
            print(f"  Average HIGH pulse: {avg_high:.1f} μs ({avg_high/1000:.3f} ms)")
            print(f"  Number of pulses:   {len(high_times)}")
            if high_times and low_times:
                period = np.mean(high_times) + np.mean(low_times)
                frequency = 1000000 / period if period > 0 else 0
                duty_cycle = (np.mean(high_times) / period * 100) if period > 0 else 0
                print(f"  Period:             {period:.1f} μs ({period/1000:.3f} ms)")
                print(f"  Frequency:          {frequency:.1f} Hz")
                print(f"  Duty cycle:         {duty_cycle:.1f} %")

def main():
    import sys
    
    # Název vstupního souboru
    if len(sys.argv) > 1:
        log_file = sys.argv[1]
    else:
        log_file = 'motor_simulation.log'
    
    # Název výstupního souboru
    if len(sys.argv) > 2:
        output_file = sys.argv[2]
    else:
        output_file = 'pwm_signals.png'
    
    print(f"Reading log file: {log_file}")
    
    try:
        # Parsování dat
        pin_data = parse_log_file(log_file)
        
        if not pin_data:
            print("Error: No pin data found in log file!")
            return 1
        
        print(f"Found {len(pin_data)} pins with data")
        
        # Výpis statistik
        print_statistics(pin_data)
        
        # Vytvoření grafu
        print(f"\nGenerating plot...")
        create_pwm_plot(pin_data, output_file)
        
        return 0
        
    except FileNotFoundError:
        print(f"Error: File '{log_file}' not found!")
        return 1
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == '__main__':
    import sys
    sys.exit(main())
