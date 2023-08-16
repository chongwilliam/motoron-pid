# Testing
from filter import ButterworthFilter
import matplotlib.pyplot as plt 
import numpy as np

sampling_rate = 1000
cutoff_freq = 1.0  # Hz
filter = ButterworthFilter(sampling_rate, cutoff_freq)

n_samples = 1000 * 10  # 10 second sample 
time = np.arange(0, n_samples) * (1 / sampling_rate)
raw_signal = np.random.normal(1, 0.1, n_samples)
filtered_signal = np.zeros(n_samples)
for i in range(n_samples):
    filtered_signal[i] = filter.update(raw_signal[i])

plt.figure()
plt.plot(time, raw_signal)
plt.plot(time, filtered_signal)
plt.show()
    