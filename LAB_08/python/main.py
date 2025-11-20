import numpy as np

# Parametry sygnału
num_samples = 256       # liczba próbek
signal_freq = 5         # częstotliwość sygnału w Hz (tylko do przykładu)
sampling_rate = 100     # częstotliwość próbkowania w Hz

# Generowanie sygnału trójkątnego
t = np.arange(num_samples) / sampling_rate
# Sygnał trójkątny: scipy.signal.sawtooth z szerokością 0.5 lub własna implementacja
triangle = 2 * np.abs(2 * (t * signal_freq - np.floor(t * signal_freq + 0.5))) - 1
signal = 2048 + 2047 * triangle  # skalowanie do zakresu 12-bit (0-4095)
signal = np.round(signal).astype(int)  # zaokrąglenie do liczb całkowitych
signal = np.clip(signal, 0, 4095)      # ograniczenie do zakresu 12-bit

# 1. Plik CSV z danymi
csv_filename = "data.csv"
np.savetxt(csv_filename, signal, fmt='%d', delimiter=',')

# 2. Plik nagłówkowy .h
header_filename = "signal.h"
header_content = f"""#ifndef SIGNAL_H
#define SIGNAL_H

#define SIGNAL_SIZE {num_samples}
extern const unsigned int signal[SIGNAL_SIZE];

#endif // SIGNAL_H
"""

with open(header_filename, 'w') as file:
    file.write(header_content)

# 3. Plik źródłowy .c
source_filename = "signal.c"
source_content = f"""#include "signal.h"
#include <stdio.h>

const unsigned int signal[SIGNAL_SIZE] = {{
"""

# Wczytanie danych z CSV do źródła
with open(csv_filename, 'r') as f:
    values = f.read().strip().split('\n')
    values = [v.strip() for v in values if v.strip()]

# Podział na linie po 16 wartości dla czytelności
lines = []
for i in range(0, len(values), 16):
    line = ", ".join(values[i:i+16])
    lines.append("    " + line)

source_content += ",\n".join(lines) + "\n};\n"

with open(source_filename, 'w') as f:
    f.write(source_content)

print("Wygenerowano pliki: data.csv, signal.h, signal.c")
