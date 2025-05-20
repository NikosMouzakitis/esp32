import sounddevice as sd
import numpy as np
import scipy.fft
import matplotlib.pyplot as plt

# Parameters
sample_rate = 44100  # Sample rate in Hz
duration = 2         # Duration of the recording in seconds

# Record audio from the microphone
print("Recording...")
audio_data = sd.rec(int(sample_rate * duration), samplerate=sample_rate, channels=1, dtype='float32')
sd.wait()
print("Recording complete.")

# Perform FFT to detect frequency
n = len(audio_data)
frequencies = np.fft.fftfreq(n, 1/sample_rate)  # Frequency array
fft_values = np.fft.fft(audio_data.flatten())  # FFT of the audio signal

# Get the magnitude of the FFT values
fft_magnitude = np.abs(fft_values)

# Find the peak frequency
peak_frequency = np.abs(frequencies[np.argmax(fft_magnitude)])

# Display the result
print(f"Dominant frequency: {peak_frequency} Hz")

# Plot the frequency spectrum
plt.figure(figsize=(10, 6))
plt.plot(frequencies[:n//2], fft_magnitude[:n//2])  # Plot positive frequencies only
plt.title("Frequency Spectrum")
plt.xlabel("Frequency (Hz)")
plt.ylabel("Magnitude")
plt.grid(True)
plt.show()

