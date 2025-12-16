
import numpy as np
import scipy.signal as signal
import os

# Configuration
# "Aggressive" means lower cutoff.
# Original: Fp=0.1. New: Fp=0.02 (5x lower cutoff).
# With Fs=200Hz, Cutoff will be ~2Hz.
NUM_TAPS = 60
CUTOFF_NORM = 0.04 # Normalized to Nyquist (0..1). 0.04 * 100Hz = 4Hz.

# Generate Coefficients (Hamming window)
taps = signal.firwin(NUM_TAPS, CUTOFF_NORM, window='hamming')

# Paths
workspace = r"c:\Users\Piotr\STM32CubeIDE\workspace_1.19.0\PUT_5_SYSTEMY_MIKROPROCESOROWE\LAB_10"
inc_file = os.path.join(workspace, "Core", "Inc", "FIR_COEFFS.h")
src_file = os.path.join(workspace, "Core", "Src", "FIR_COEFFS.c")

# Write Header
with open(inc_file, 'w') as f:
    f.write("#ifndef FIR_COEFFS_H_\n#define FIR_COEFFS_H_\n\n")
    f.write('#include "arm_math.h"\n\n')
    f.write(f"#define NUM_TAPS {NUM_TAPS}\n")
    f.write("extern float32_t FIR_COEFFS[NUM_TAPS];\n\n")
    f.write("#endif // FIR_COEFFS_H_\n")

# Write Source
with open(src_file, 'w') as f:
    f.write('#include "FIR_COEFFS.h"\n\n')
    f.write(f"float32_t FIR_COEFFS[{NUM_TAPS}] = {{\n")
    for i, val in enumerate(taps):
        f.write(f"  {val:.9f}f")
        if i < NUM_TAPS - 1:
            f.write(",\n")
        else:
            f.write("\n")
    f.write("};\n")

print(f"Generated {NUM_TAPS} coeffs with cutoff {CUTOFF_NORM} to {src_file}")
