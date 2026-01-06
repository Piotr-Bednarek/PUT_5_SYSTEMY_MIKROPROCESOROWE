"""
Generator danych testowych dla regulatora PID - arm_pid_f32 CMSIS DSP

Obiekt sterowania: Inercyjny I rzędu
- Transmitancja: G(s) = K/(T*s + 1)
- K = 10 (wzmocnienie)
- T = 0.5s (stała czasowa)

Metoda doboru nastaw: Ziegler-Nichols (metoda eksperymentalna uproszczona)
lub metoda SIMC dla obiektu I rzędu
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import os

# ============================================================================
# PARAMETRY SYMULACJI
# ============================================================================
# Okres próbkowania
Ts = 0.01  # 10ms = 100Hz

# Czas symulacji
t_sim = 5.0  # 5 sekund
N = int(t_sim / Ts)
t = np.arange(N) * Ts

# Parametry obiektu I rzędu: G(s) = K/(T*s + 1)
K_plant = 10.0
T_plant = 0.5

# ============================================================================
# DYSKRETYZACJA OBIEKTU (metoda Eulera)
# ============================================================================
# Równanie ciągłe: T*dy/dt + y = K*u
# Dyskretne: y[k] = a*y[k-1] + b*u[k-1]
a = 1 - Ts/T_plant
b = K_plant * Ts/T_plant

print("="*60)
print("PARAMETRY OBIEKTU (DYSKRETYZACJA)")
print("="*60)
print(f"Okres próbkowania Ts = {Ts} s")
print(f"Współczynnik a = {a:.6f}")
print(f"Współczynnik b = {b:.6f}")

# ============================================================================
# DOBÓR NASTAW PID - METODA SIMC (Simple Internal Model Control)
# ============================================================================
# Dla obiektu I rzędu: G(s) = K/(T*s + 1)
# Metoda SIMC (zalecana dla obiektów bez opóźnienia):
# tau_c = T_plant  (czas zamknięcia pętli, można dobrać tau_c = T_plant lub mniejsze)

tau_c = T_plant  # Można zmniejszyć dla szybszej odpowiedzi (np. T_plant/3)

# Nastawy SIMC:
Kp = T_plant / (K_plant * tau_c)
Ti = min(T_plant, 4*tau_c)  # Czas zdwojenia
Td = 0  # Dla obiektu I rzędu Td=0

Ki = Kp / Ti if Ti > 0 else 0
Kd = Kp * Td

# ============================================================================
# PRZELICZENIE NA WSPÓŁCZYNNIKI arm_pid_f32 (ALGORYTM PRZYROSTOWY)
# ============================================================================
# arm_pid_f32 używa algorytmu przyrostowego:
# y[n] = y[n-1] + A0 * (x[n] - x[n-1]) + A1 * x[n] + A2 * (x[n] - 2*x[n-1] + x[n-2])
# 
# Gdzie x[n] = uchyb
# Współczynniki:
A0 = Kp + Ki*Ts + Kd/Ts
A1 = -Kp - 2*Kd/Ts
A2 = Kd/Ts

print("\n" + "="*60)
print("NASTAWY REGULATORA PID")
print("="*60)
print(f"Kp (wzmocnienie proporcjonalne) = {Kp:.6f}")
print(f"Ki (wzmocnienie całkujące) = {Ki:.6f}")
print(f"Kd (wzmocnienie różniczkujące) = {Kd:.6f}")
print(f"\nWspółczynniki arm_pid_f32 (przyrostowe):")
print(f"A0 = {A0:.6f}")
print(f"A1 = {A1:.6f}")
print(f"A2 = {A2:.6f}")

# ============================================================================
# SYGNAŁ REFERENCYJNY (SETPOINT)
# ============================================================================
# Skok jednostkowy w t=0.5s od 0 do 1
setpoint = np.zeros(N)
setpoint[int(0.5/Ts):] = 1.0

# ============================================================================
# SYMULACJA REFERENCYJNA (Python) - ALGORYTM arm_pid_f32
# ============================================================================
print("\n" + "="*60)
print("SYMULACJA REFERENCYJNA (arm_pid_f32)")
print("="*60)

y_ref = np.zeros(N)  # Wyjście obiektu
u_ref = np.zeros(N)  # Sygnał sterujący
e_ref = np.zeros(N)  # Uchyb

# Zmienne stanu arm_pid_f32 (algorytm przyrostowy)
state = np.zeros(3)  # [e[n-2], e[n-1], u[n-1]]

# Symulacja krok po kroku
for k in range(N):
    # Uchyb
    e_ref[k] = setpoint[k] - y_ref[k]
    
    # Algorytm przyrostowy arm_pid_f32:
    # u[n] = u[n-1] + A0*(e[n] - e[n-1]) + A1*e[n] + A2*(e[n] - 2*e[n-1] + e[n-2])
    
    if k == 0:
        # Pierwsze wywołanie
        u_ref[k] = A0 * e_ref[k]
    elif k == 1:
        # Drugie wywołanie (brak e[n-2])
        u_ref[k] = state[2] + A0 * (e_ref[k] - state[1]) + A1 * e_ref[k]
    else:
        # Pełny algorytm
        u_ref[k] = state[2] + A0 * (e_ref[k] - state[1]) + A1 * e_ref[k] + A2 * (e_ref[k] - 2*state[1] + state[0])
    
    # Saturacja 0-10
    u_ref[k] = np.clip(u_ref[k], 0, 10)
    
    # Aktualizacja stanu arm_pid_f32
    state[0] = state[1]  # e[n-2] = e[n-1]
    state[1] = e_ref[k]  # e[n-1] = e[n]
    state[2] = u_ref[k]  # u[n-1] = u[n]
    
    # Obiekt (model dyskretny)
    if k > 0:
        y_ref[k] = a * y_ref[k-1] + b * u_ref[k-1]

print(f"Maksymalna wartość sygnału sterującego: {np.max(u_ref):.3f}")
print(f"Ustalona wartość wyjścia: {y_ref[-1]:.3f}")
print(f"Przeregulowanie: {(np.max(y_ref) - 1.0)*100:.1f}%")

# ============================================================================
# GENEROWANIE PLIKÓW C DLA STM32
# ============================================================================
print("\n" + "="*60)
print("GENEROWANIE PLIKÓW C")
print("="*60)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_DIR = os.path.join(SCRIPT_DIR, "Core", "Src")
INC_DIR = os.path.join(SCRIPT_DIR, "Core", "Inc")

os.makedirs(SRC_DIR, exist_ok=True)
os.makedirs(INC_DIR, exist_ok=True)

def write_array_files(name, data, extra_defines=None):
    """Generuje plik .h i .c dla tablicy"""
    h_file = os.path.join(INC_DIR, f"{name}.h")
    c_file = os.path.join(SRC_DIR, f"{name}.c")
    
    guard = f"{name.upper()}_H_"
    
    # Header
    with open(h_file, 'w') as f:
        f.write(f"#ifndef {guard}\n")
        f.write(f"#define {guard}\n\n")
        f.write('#include "arm_math.h"\n\n')
        
        if extra_defines:
            for key, val in extra_defines.items():
                f.write(f"#define {key} {val}\n")
            f.write("\n")
        
        f.write(f"extern float32_t {name.upper()}[{len(data)}];\n\n")
        f.write(f"#endif // {guard}\n")
    
    # Source
    with open(c_file, 'w') as f:
        f.write(f'#include "{name}.h"\n\n')
        f.write(f"float32_t {name.upper()}[{len(data)}] = {{\n")
        for i, val in enumerate(data):
            f.write(f"  {val:.9f}f")
            if i < len(data) - 1:
                f.write(",\n")
            else:
                f.write("\n")
        f.write("};\n")
    
    print(f"Utworzono: {h_file}")
    print(f"Utworzono: {c_file}")

# Generuj pliki
write_array_files("pid_setpoint", setpoint, {"PID_TEST_LENGTH": N})
write_array_files("pid_ref_error", e_ref)
write_array_files("pid_ref_output", u_ref)

# Plik z nastawami PID
pid_params_h = os.path.join(INC_DIR, "pid_params.h")
with open(pid_params_h, 'w') as f:
    f.write("#ifndef PID_PARAMS_H_\n")
    f.write("#define PID_PARAMS_H_\n\n")
    f.write("// Współczynniki arm_pid_f32 (algorytm przyrostowy)\n")
    f.write(f"#define PID_A0 {A0:.9f}f\n")
    f.write(f"#define PID_A1 {A1:.9f}f\n")
    f.write(f"#define PID_A2 {A2:.9f}f\n\n")
    f.write("// Nastawy PID (dla informacji)\n")
    f.write(f"#define PID_KP {Kp:.9f}f\n")
    f.write(f"#define PID_KI {Ki:.9f}f\n")
    f.write(f"#define PID_KD {Kd:.9f}f\n\n")
    f.write("// Parametry obiektu\n")
    f.write(f"#define PLANT_A {a:.9f}f\n")
    f.write(f"#define PLANT_B {b:.9f}f\n\n")
    f.write("#endif // PID_PARAMS_H_\n")

print(f"Utworzono: {pid_params_h}")

# ============================================================================
# WIZUALIZACJA
# ============================================================================
print("\n" + "="*60)
print("GENEROWANIE WYKRESÓW")
print("="*60)

fig, axes = plt.subplots(3, 1, figsize=(12, 10))

# Wykres 1: Setpoint i wyjście
axes[0].plot(t, setpoint, 'k--', linewidth=2, label='Setpoint (wartość zadana)')
axes[0].plot(t, y_ref, 'b-', linewidth=2, label='Wyjście obiektu y(t)')
axes[0].set_xlabel('Czas [s]')
axes[0].set_ylabel('Wartość')
axes[0].set_title('Odpowiedź skokowa obiektu z regulatorem PID')
axes[0].legend()
axes[0].grid(True)

# Wykres 2: Uchyb
axes[1].plot(t, e_ref, 'r-', linewidth=2)
axes[1].set_xlabel('Czas [s]')
axes[1].set_ylabel('Uchyb e(t)')
axes[1].set_title('Uchyb regulacji')
axes[1].grid(True)
axes[1].axhline(y=0, color='k', linestyle='--', linewidth=0.5)

# Wykres 3: Sygnał sterujący
axes[2].plot(t, u_ref, 'g-', linewidth=2)
axes[2].set_xlabel('Czas [s]')
axes[2].set_ylabel('Sygnał sterujący u(t)')
axes[2].set_title('Sygnał sterujący (wyjście regulatora PID)')
axes[2].grid(True)

plt.tight_layout()
plot_path = os.path.join(SCRIPT_DIR, "pid_simulation.png")
plt.savefig(plot_path, dpi=150)
print(f"Utworzono: {plot_path}")

print("\n" + "="*60)
print("GENERACJA ZAKOŃCZONA POMYŚLNIE!")
print("="*60)
print(f"\nWygenerowano {N} próbek ({t_sim}s)")
print("\nNastępne kroki:")
print("1. Skompiluj projekt STM32")
print("2. Uruchom test jednostkowy arm_pid_f32")
print("3. Porównaj RMSE sygnału sterującego")
print("4. Kryterium sukcesu: RMSE < 1e-6")
