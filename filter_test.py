# Testing
from filter import ButterworthFilter
import matplotlib.pyplot as plt 
import numpy as np

sampling_rate = 1
cutoff_freq = 0.1  # Hz
filter = ButterworthFilter(sampling_rate, cutoff_freq)

n_samples = 1000
raw_signal = np.random.normal(1, 1, n_samples)
filtered_signal = np.zeros(n_samples)
for i in range(n_samples):
    filtered_signal[i] = filter.update(raw_signal[i])

plt.figure()
plt.plot(raw_signal)
plt.plot(filtered_signal)
plt.show()
    