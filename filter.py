# Low-pass butterworth filter for 1D signal 
import numpy as np 

class ButterworthFilter:
    def __init__(self, sampling_rate, cutoff_freq):
        self.order = 2
        self.raw_buffer = np.zeros(self.order + 1)
        self.filtered_buffer = np.zeros(self.order)
        self.raw_coeff = np.zeros(self.order + 1)
        self.filtered_coeff = np.zeros(self.order)

        self.setCutoffFrequency(sampling_rate, cutoff_freq)

    def setCutoffFrequency(self, sampling_rate, cutoff_freq):
        fc = cutoff_freq / sampling_rate
        if fc >= 0.5 or fc < 0:
            raise Exception('Normalized cutoff frequency should be between 0 and 0.5')

        pre_warp_cutoff = np.tan(np.pi * fc)

        gain = (1 / (pre_warp_cutoff * pre_warp_cutoff) + np.sqrt(2) / pre_warp_cutoff + 1)

        ita = 1.0 / np.tan(np.pi * fc)
        q = np.sqrt(2.0);

        self.raw_coeff = np.array([1, 2, 1]) / gain
        self.filtered_coeff = np.array([2 - 2 / (pre_warp_cutoff * pre_warp_cutoff), 
                                        1 / (pre_warp_cutoff * pre_warp_cutoff) - np.sqrt(2) / pre_warp_cutoff + 1]) / gain

    def update(self, value):
        for i in range(self.order, 0, -1):
            self.raw_buffer[i] = self.raw_buffer[i - 1]
        self.raw_buffer[0] = value
        y = np.dot(self.raw_buffer, self.raw_coeff) - np.dot(self.filtered_buffer, self.filtered_coeff)
        for i in range(self.order - 1, 0, -1):
            self.filtered_buffer[i] = self.filtered_buffer[i - 1]
        self.filtered_buffer[0] = y
        return y


