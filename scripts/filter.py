import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, freqz

# 1. Define Filter Parameters
fs = 200.0          # Sampling frequency
cutoff = 15.0       # Cutoff frequency 
order = 2           # 2nd-order Butterworth filter

b, a = butter(order, cutoff, fs=fs, btype='low', analog=False)
print("--- Filter Coefficients ---")
print(f"b (Numerator):   {b}")
print(f"a (Denominator): {a}\n")

# 3. Calculate the Frequency Response
# freqz computes the response at 512 points up to the Nyquist frequency (200/2 = 100 Hz)
# since whole = False
w, h = freqz(b, a, whole=False, fs=fs)
magnitude_db = 20 * np.log10(np.abs(h))
phase_degrees = np.angle(h, deg=True)

# 4. Plot the Bode Plot
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

# Subplot 1: Magnitude Response
ax1.plot(w, magnitude_db, color='crimson', linewidth=2)
ax1.axvline(cutoff, color='black', linestyle='--', label=f'Cutoff Frequency ({cutoff} Hz)')
ax1.axhline(-3, color='gray', linestyle=':', label='-3 dB Point')
ax1.set_title("Bode Plot - 2nd-Order Butterworth Low-Pass Filter")
ax1.set_ylabel("Magnitude Response [dB]")
ax1.set_ylim(-40, 5) # Focus on the passband and stopband drop
ax1.grid(True, which='both', axis='both')
ax1.legend(loc="lower left")

# Subplot 2: Phase Response (The Phase Lag Rule!)
ax2.plot(w, phase_degrees, color='royalblue', linewidth=2)
ax2.axvline(cutoff, color='black', linestyle='--')
ax2.set_xlabel("Frequency [Hz]")
ax2.set_ylabel("Phase Shift [Degrees]")
ax2.set_xlim(0, fs / 2) # X-axis cuts off at Nyquist (100 Hz)
ax2.grid(True, which='both', axis='both')

plt.tight_layout()
plt.show()