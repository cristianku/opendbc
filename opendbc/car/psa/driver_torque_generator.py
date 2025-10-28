import numpy as np
import random
from scipy.signal.windows import gaussian

class DriverTorqueGenerator:
    def __init__(self):
        self.sequence = []
        self.index = 0

    def reset(self):
      self.sequence = []
      self.index = 0

    def _generate_sequence(self):
        duration = 200
        std = duration / 6
        peak_torque = random.randint(15, 20)

        seq = peak_torque * gaussian(duration, std=std)
        seq = seq / np.max(seq) * peak_torque

        if random.random() < 0.2:
            opposite_peak = random.randint(12, 18)
            opposite_seq = -opposite_peak * gaussian(100, std=100/6)
            seq = np.concatenate([seq, np.zeros(2), opposite_seq])

        return np.round(seq).astype(int).tolist()

    def next_value(self):
        if self.index >= len(self.sequence):
            self.sequence = self._generate_sequence()
            self.index = 0

        val = self.sequence[self.index]
        self.index += 1
        return val
