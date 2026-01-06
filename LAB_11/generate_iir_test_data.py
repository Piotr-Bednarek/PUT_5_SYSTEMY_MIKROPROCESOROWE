"""
IIR Filter Test Data Generator for CMSIS DSP arm_biquad_cascade_df1_f32
Generates:
- IIR filter coefficients (Butterworth 4th order, 2 biquad sections)
- Test signal (1000 samples: sine wave + noise)
- Reference filtered output
- C header and source files for STM32
"""

import numpy as np
from scipy import signal
import matplotlib.pyplot as plt
import os

# ============================================================================
# PARAMETERS
# ============================================================================
FILTER_ORDER = 4           # Filter order (must be even for biquad cascade)
CUTOFF_FREQ = 10.0         # Cutoff frequency [Hz]
SAMPLING_FREQ = 200.0      # Sampling frequency [Hz]
NUM_SAMPLES = 1000         # Number of test signal samples

# Test signal parameters
SIGNAL_FREQ = 5.0          # Low frequency component [Hz] - should pass
NOISE_FREQ = 50.0          # High frequency component [Hz] - should be filtered
NOISE_AMPLITUDE = 0.3      # Noise amplitude relative to signal

# Paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_DIR = os.path.join(SCRIPT_DIR, "Core", "Src")
INC_DIR = os.path.join(SCRIPT_DIR, "Core", "Inc")

# Create directories if they don't exist
os.makedirs(SRC_DIR, exist_ok=True)
os.makedirs(INC_DIR, exist_ok=True)

# ============================================================================
# 1. DESIGN IIR FILTER (Butterworth)
# ============================================================================
print("="*60)
print("IIR FILTER DESIGN - BUTTERWORTH")
print("="*60)

# Normalized frequency (Nyquist = 1)
nyquist = SAMPLING_FREQ / 2
normalized_cutoff = CUTOFF_FREQ / nyquist

print(f"Filter Order: {FILTER_ORDER}")
print(f"Cutoff Frequency: {CUTOFF_FREQ} Hz")
print(f"Sampling Frequency: {SAMPLING_FREQ} Hz")
print(f"Normalized Cutoff: {normalized_cutoff:.4f}")

# Design Butterworth filter and get second-order sections (SOS)
# SOS format: each row is [b0, b1, b2, a0, a1, a2]
sos = signal.butter(FILTER_ORDER, normalized_cutoff, btype='low', 
                    analog=False, output='sos')

num_stages = sos.shape[0]
print(f"Number of Biquad Stages: {num_stages}")
print(f"\nSOS Coefficients:")
print(sos)

# Convert to CMSIS DSP format: [b0, b1, b2, a1, a2] (a0 is normalized to 1)
# CMSIS expects coefficients in the order: b0, b1, b2, -a1, -a2
coeffs_cmsis = []
for i in range(num_stages):
    b0, b1, b2, a0, a1, a2 = sos[i]
    # Normalize by a0 (should already be 1, but just in case)
    b0_norm = b0 / a0
    b1_norm = b1 / a0
    b2_norm = b2 / a0
    a1_norm = a1 / a0
    a2_norm = a2 / a0
    
    # CMSIS DSP uses negated a coefficients
    coeffs_cmsis.extend([b0_norm, b1_norm, b2_norm, -a1_norm, -a2_norm])

print(f"\nCMSIS DSP Coefficients (flattened):")
print(coeffs_cmsis)

# ============================================================================
# 2. GENERATE TEST SIGNAL
# ============================================================================
print("\n" + "="*60)
print("TEST SIGNAL GENERATION")
print("="*60)

t = np.arange(NUM_SAMPLES) / SAMPLING_FREQ
test_signal = (np.sin(2 * np.pi * SIGNAL_FREQ * t) + 
               NOISE_AMPLITUDE * np.sin(2 * np.pi * NOISE_FREQ * t) +
               0.1 * np.random.randn(NUM_SAMPLES))

print(f"Generated {NUM_SAMPLES} samples")
print(f"Signal component: {SIGNAL_FREQ} Hz")
print(f"Noise component: {NOISE_FREQ} Hz")

# ============================================================================
# 3. REFERENCE FILTERING (using scipy)
# ============================================================================
print("\n" + "="*60)
print("REFERENCE FILTERING")
print("="*60)

ref_output = signal.sosfilt(sos, test_signal)

print(f"Filtered {len(ref_output)} samples")
print(f"Input RMS: {np.sqrt(np.mean(test_signal**2)):.6f}")
print(f"Output RMS: {np.sqrt(np.mean(ref_output**2)):.6f}")

# ============================================================================
# 4. GENERATE C FILES
# ============================================================================
print("\n" + "="*60)
print("GENERATING C FILES")
print("="*60)

def write_header(filename, guard_name, array_name, array_size, extra_defines=None):
    """Write C header file"""
    with open(filename, 'w') as f:
        f.write(f"#ifndef {guard_name}_H_\n")
        f.write(f"#define {guard_name}_H_\n\n")
        f.write('#include "arm_math.h"\n\n')
        
        if extra_defines:
            for define_name, define_value in extra_defines.items():
                f.write(f"#define {define_name} {define_value}\n")
            f.write("\n")
        
        f.write(f"extern float32_t {array_name}[{array_size}];\n\n")
        f.write(f"#endif // {guard_name}_H_\n")

def write_source(filename, header_name, array_name, data):
    """Write C source file"""
    with open(filename, 'w') as f:
        f.write(f'#include "{header_name}"\n\n')
        f.write(f"float32_t {array_name}[{len(data)}] = {{\n")
        
        for i, val in enumerate(data):
            f.write(f"  {val:.9f}f")
            if i < len(data) - 1:
                f.write(",\n")
            else:
                f.write("\n")
        
        f.write("};\n")

# IIR_COEFFS files
iir_h = os.path.join(INC_DIR, "IIR_COEFFS.h")
iir_c = os.path.join(SRC_DIR, "IIR_COEFFS.c")

write_header(iir_h, "IIR_COEFFS", "IIR_COEFFS", len(coeffs_cmsis),
             extra_defines={"NUM_STAGES": num_stages})
write_source(iir_c, "IIR_COEFFS.h", "IIR_COEFFS", coeffs_cmsis)
print(f"Created: {iir_h}")
print(f"Created: {iir_c}")

# TEST_SIGNAL files
test_h = os.path.join(INC_DIR, "TEST_SIGNAL.h")
test_c = os.path.join(SRC_DIR, "TEST_SIGNAL.c")

write_header(test_h, "TEST_SIGNAL", "TEST_SIGNAL", NUM_SAMPLES,
             extra_defines={"SIGNAL_LENGTH": NUM_SAMPLES})
write_source(test_c, "TEST_SIGNAL.h", "TEST_SIGNAL", test_signal)
print(f"Created: {test_h}")
print(f"Created: {test_c}")

# REF_SIGNAL files
ref_h = os.path.join(INC_DIR, "REF_SIGNAL.h")
ref_c = os.path.join(SRC_DIR, "REF_SIGNAL.c")

write_header(ref_h, "REF_SIGNAL", "REF_SIGNAL", NUM_SAMPLES,
             extra_defines={"SIGNAL_LENGTH": NUM_SAMPLES})
write_source(ref_c, "REF_SIGNAL.h", "REF_SIGNAL", ref_output)
print(f"Created: {ref_h}")
print(f"Created: {ref_c}")

# ============================================================================
# 5. VISUALIZATION
# ============================================================================
print("\n" + "="*60)
print("GENERATING PLOTS")
print("="*60)

# Plot 1: Filter frequency response
fig, axes = plt.subplots(2, 1, figsize=(10, 8))

w, h = signal.sosfreqz(sos, worN=2048, fs=SAMPLING_FREQ)

# Magnitude
axes[0].plot(w, 20 * np.log10(abs(h)), 'b', linewidth=2)
axes[0].axvline(CUTOFF_FREQ, color='r', linestyle='--', label=f'Cutoff: {CUTOFF_FREQ} Hz')
axes[0].set_xlabel('Frequency [Hz]')
axes[0].set_ylabel('Magnitude [dB]')
axes[0].set_title(f'Butterworth Lowpass Filter - Order {FILTER_ORDER}')
axes[0].grid(True)
axes[0].legend()
axes[0].set_xlim([0, SAMPLING_FREQ/2])

# Phase
axes[1].plot(w, np.angle(h) * 180 / np.pi, 'b', linewidth=2)
axes[1].axvline(CUTOFF_FREQ, color='r', linestyle='--', label=f'Cutoff: {CUTOFF_FREQ} Hz')
axes[1].set_xlabel('Frequency [Hz]')
axes[1].set_ylabel('Phase [degrees]')
axes[1].set_title('Phase Response')
axes[1].grid(True)
axes[1].legend()
axes[1].set_xlim([0, SAMPLING_FREQ/2])

plt.tight_layout()
plot1_path = os.path.join(SCRIPT_DIR, "filter_response.png")
plt.savefig(plot1_path, dpi=150)
print(f"Created: {plot1_path}")

# Plot 2: Signals comparison
fig, axes = plt.subplots(3, 1, figsize=(12, 10))

# Time domain - first 200 samples for clarity
n_plot = min(200, NUM_SAMPLES)
t_plot = t[:n_plot]

axes[0].plot(t_plot, test_signal[:n_plot], 'b', linewidth=1, alpha=0.7)
axes[0].set_xlabel('Time [s]')
axes[0].set_ylabel('Amplitude')
axes[0].set_title('Test Signal (Input)')
axes[0].grid(True)

axes[1].plot(t_plot, ref_output[:n_plot], 'r', linewidth=1.5)
axes[1].set_xlabel('Time [s]')
axes[1].set_ylabel('Amplitude')
axes[1].set_title('Filtered Signal (Output)')
axes[1].grid(True)

axes[2].plot(t_plot, test_signal[:n_plot], 'b', linewidth=1, alpha=0.5, label='Input')
axes[2].plot(t_plot, ref_output[:n_plot], 'r', linewidth=1.5, label='Output')
axes[2].set_xlabel('Time [s]')
axes[2].set_ylabel('Amplitude')
axes[2].set_title('Input vs Output Comparison')
axes[2].grid(True)
axes[2].legend()

plt.tight_layout()
plot2_path = os.path.join(SCRIPT_DIR, "signals_comparison.png")
plt.savefig(plot2_path, dpi=150)
print(f"Created: {plot2_path}")

# Plot 3: FFT comparison
fig, ax = plt.subplots(1, 1, figsize=(10, 6))

# Compute FFT
fft_input = np.fft.rfft(test_signal)
fft_output = np.fft.rfft(ref_output)
freqs = np.fft.rfftfreq(NUM_SAMPLES, 1/SAMPLING_FREQ)

ax.plot(freqs, 20*np.log10(np.abs(fft_input)), 'b', alpha=0.7, label='Input FFT')
ax.plot(freqs, 20*np.log10(np.abs(fft_output)), 'r', linewidth=2, label='Output FFT')
ax.axvline(CUTOFF_FREQ, color='g', linestyle='--', label=f'Cutoff: {CUTOFF_FREQ} Hz')
ax.set_xlabel('Frequency [Hz]')
ax.set_ylabel('Magnitude [dB]')
ax.set_title('Frequency Domain Comparison')
ax.grid(True)
ax.legend()
ax.set_xlim([0, SAMPLING_FREQ/2])

plt.tight_layout()
plot3_path = os.path.join(SCRIPT_DIR, "fft_comparison.png")
plt.savefig(plot3_path, dpi=150)
print(f"Created: {plot3_path}")

print("\n" + "="*60)
print("GENERATION COMPLETE!")
print("="*60)
print(f"\nGenerated files:")
print(f"  - {num_stages} biquad stages ({FILTER_ORDER}th order filter)")
print(f"  - {NUM_SAMPLES} test samples")
print(f"  - 6 C files (.h and .c for coefficients, test signal, reference)")
print(f"  - 3 visualization plots")
print("\nNext steps:")
print("  1. Build the STM32 project with these files")
print("  2. Run the unit test on the microcontroller")
print("  3. Verify UART output shows 'test passed!' (RMSE < 1e-6)")
