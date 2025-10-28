import numpy as np
import random

def _gaussian_window(length, std):
    """Crea una finestra gaussiana normalizzata (sostituto di scipy.signal.windows.gaussian)."""
    x = np.arange(length)
    center = (length - 1) / 2.0
    return np.exp(-0.5 * ((x - center) / std) ** 2)

class DriverTorqueGenerator:
    def __init__(self):
        self.sequence = []
        self.index = 0

    def reset(self):
        self.sequence = []
        self.index = 0

    def _generate_sequence(self):
        duration = 200  # 200 frame @100Hz = 2s
        std = duration / 6
        peak_torque = random.randint(15, 20)

        # curva principale (sostituto di gaussian)
        seq = _gaussian_window(duration, std)
        seq = seq / np.max(seq) * peak_torque

        # 20% di probabilità: impulso opposto
        if random.random() < 0.2:
            opposite_peak = random.randint(12, 18)
            opposite_seq = -opposite_peak * _gaussian_window(100, 100 / 6)
            seq = np.concatenate([seq, np.zeros(2), opposite_seq])

        return np.round(seq).astype(int).tolist()

    def next_value(self):
        if self.index >= len(self.sequence):
            self.sequence = self._generate_sequence()
            self.index = 0
        val = self.sequence[self.index]
        self.index += 1
        return val
