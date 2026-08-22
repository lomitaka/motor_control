#!/usr/bin/env python3

import sys
import csv
import matplotlib.pyplot as plt


filename = sys.argv[1] if len(sys.argv) > 1 else "motor_logs.log"

with open(filename, newline="") as f:
    reader = csv.DictReader(f)
    rows = list(reader)

# První sloupec = čas v mikrosekundách
time = [float(row["time_us"])/1000 for row in rows]

# Ostatní sloupce = jednotlivé křivky
for column in reader.fieldnames:
    if column == "time_us":
        continue

    values = [float(row[column]) for row in rows]
    plt.plot(time, values, label=column)

plt.xlabel("Time [ms]")
plt.ylabel("Value")
plt.title(filename)

plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()