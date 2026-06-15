import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

csv_path = "vicon_room1/vicon_room1/V1_01_easy/V1_01_easy/mav0/imu0/data.csv"
df = pd.read_csv(csv_path)

timestamps_ns = df.iloc[:, 0].to_numpy()
time = (timestamps_ns - timestamps_ns[0]) / 1e9  
fs = 200.0  # 200 Hz sampling frequency
dt = 1.0 / fs

imu_magnitudes = []
imu_frequencies = []

# Define standard labels for the EuRoC data columns
axis_names = ["w_x (Rad/s)", "w_y (Rad/s)", "w_z (Rad/s)", "a_x (m/s^2)", "a_y (m/s^2)", "a_z (m/s^2)"]

# 1. Math Loop: Compute FFT for all 6 axes
for i in range(1, 7):
    signal = df.iloc[:, i].to_numpy()
    N = len(signal)
    fft_values = np.fft.rfft(signal)
    frequencies = np.fft.rfftfreq(N, d=dt)
    magnitudes = np.abs(fft_values) / N
    magnitudes[1:] *= 2.0

    imu_magnitudes.append(magnitudes)
    imu_frequencies.append(frequencies)

# 2. Plotting Loop: Map to a 6x2 grid
# Increased height to 18 so 6 rows of plots are highly readable
plt.figure(figsize=(14, 18))

for i in range(6):
    signal = df.iloc[:, i + 1].to_numpy()
    
    # Column 1: Time Domain (Slots: 1, 3, 5, 7, 9, 11)
    plt.subplot(6, 2, 2 * i + 1)
    plt.plot(time, signal, color='royalblue', linewidth=0.5)
    plt.title(f"{axis_names[i]} - Time Domain")
    plt.xlabel("Time [seconds]")
    plt.ylabel("Raw Amplitude")
    plt.grid(True)

    # Column 2: Frequency Domain (Slots: 2, 4, 6, 8, 10, 12)
    plt.subplot(6, 2, 2 * i + 2)
    frequencies = imu_frequencies[i]
    magnitudes = imu_magnitudes[i]
    
    plt.plot(frequencies, magnitudes, color='crimson', linewidth=0.8)
    plt.title(f"{axis_names[i]} - FFT Spectrum")
    plt.xlabel("Frequency [Hz]")
    plt.ylabel("Magnitude")
    plt.xlim(-1, fs / 2)  # Strictly lock from 0 to Nyquist (100 Hz)
    plt.grid(True)

# 3. Render Dashboard (Outside the loop!)
plt.tight_layout()
plt.show()