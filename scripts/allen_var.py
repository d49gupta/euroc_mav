import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
import allan_variance as av

# 1. Load Data
csv_path = "vicon_room1/vicon_room1/V1_01_easy/V1_01_easy/mav0/imu0/data.csv"
df = pd.read_csv(csv_path)

raw_signal = df.iloc[:, 5].to_numpy()
fs = 200.0  # Sampling rate
dt = 1.0 / fs
cutoff = 15.0
order = 2
b, a = butter(order, cutoff, fs=fs, btype='low', analog=False)
filtered_signal = filtfilt(b, a, raw_signal)

tau, avar = av.compute_avar(raw_signal, dt)
params, avar_pred = av.estimate_parameters(tau, avar)
print(f"Estimated Raw Noise Parameters: {params}")

filt_tau, filt_avar = av.compute_avar(filtered_signal, dt)
filt_params, filt_avar_pred = av.estimate_parameters(filt_tau, filt_avar)
print(f"Estimated Filtered Noise Parameters: {filt_params}")

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

ax1.loglog(tau, avar, '.', color='crimson', label='Raw Signal')
ax1.set_title("Allan Deviation: Raw Signal")
ax1.set_xlabel("Averaging time, s")
ax1.set_ylabel("ADEV")
ax1.grid(True, which="both", ls="-", color='0.85')
ax1.legend()

ax2.loglog(filt_tau, filt_avar, '.', color='royalblue', label='Filtered Signal')
ax2.set_title("Allan Deviation: Filtered (15 Hz Cutoff)")
ax2.set_xlabel("Averaging time, s")
ax2.set_ylabel("ADEV")
ax2.grid(True, which="both", ls="-", color='0.85')
ax2.legend()

plt.tight_layout()
plt.show()
