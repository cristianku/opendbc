import numpy as np
import random


def _gaussian_window(length, std):
    """Crea una finestra gaussiana normalizzata (sostituto di scipy.signal.windows.gaussian)."""
    x = np.arange(length)
    center = (length - 1) / 2.0
    std = max(std, 1.0)  # Evita std troppo piccolo
    return np.exp(-0.5 * ((x - center) / std) ** 2)


class DriverTorqueGenerator:
    """
    Generates a simulated driver torque signal based on a Gaussian curve.
    Parameters are dynamically configurable from the controller for maximum flexibility.
    """
    def __init__(self, duration_range=(600, 800), std_divisor=4,
                 peak_torque_range=(12, 18), opposite_probability=0.2,
                 rate_hz=100, seed=None, eps_max=20):
        """
        Initialize the generator with configurable parameters.

        Args:
            duration_range: (min, max) frames for main Gaussian curve
            std_divisor: controls curve width (higher = narrower, lower = wider)
            peak_torque_range: (min, max) peak torque in Nm
            opposite_probability: probability of adding an opposite torque pulse
            rate_hz: control loop frequency (default 100 Hz)
            seed: random seed for reproducibility in testing (None = random)
            eps_max: maximum EPS torque limit for saturation
        """
        self.rate_hz = rate_hz
        self.duration_range = duration_range
        self.std_divisor = std_divisor
        self.peak_torque_range = peak_torque_range
        self.opposite_probability = opposite_probability
        self.eps_max = eps_max

        # Random generator locale per riproducibilità
        self.rng = random.Random(seed) if seed is not None else random

        self.sequence = []
        self.index = 0

    def reset(self, **kwargs):
        """
        Reset the sequence and optionally update parameters for the next generation.
        Pre-generates the sequence so is_active() returns True immediately.
        Validates all input ranges to prevent edge-cases.
        """
        if 'duration_range' in kwargs:
            self.duration_range = kwargs['duration_range']
        if 'std_divisor' in kwargs:
            self.std_divisor = kwargs['std_divisor']
        if 'peak_torque_range' in kwargs:
            self.peak_torque_range = kwargs['peak_torque_range']
        if 'opposite_probability' in kwargs:
            self.opposite_probability = kwargs['opposite_probability']
        if 'rate_hz' in kwargs:
            self.rate_hz = kwargs['rate_hz']
        if 'eps_max' in kwargs:
            self.eps_max = kwargs['eps_max']

        # Validazione duration_range
        a, b = self.duration_range
        if a < 1:
            a = 1
        if b < a:
            b = a
        self.duration_range = (a, b)

        # Validazione peak_torque_range
        p0, p1 = self.peak_torque_range
        if p1 < p0:
            p1 = p0
        self.peak_torque_range = (int(p0), int(p1))

        # Validazione std_divisor
        if self.std_divisor <= 0:
            self.std_divisor = 4.0

        # Validazione opposite_probability
        self.opposite_probability = max(0.0, min(1.0, self.opposite_probability))

        # Pre-genera la sequenza così is_active() ritorna subito True
        self.sequence = self._generate_sequence()
        self.index = 0

    def is_active(self):
        """
        Check if we're currently in the middle of a sequence.
        Returns True if there are still values to output, False if sequence is complete.
        """
        return len(self.sequence) > 0 and self.index < len(self.sequence)

    def is_done(self):
        """
        Check if the sequence has been fully consumed.
        Returns True when all values have been read and sequence is finished.
        Use this in the controller to detect burst completion.
        """
        return len(self.sequence) > 0 and self.index >= len(self.sequence)

    def get_sequence_length(self):
        """Return the total length of the current sequence in frames."""
        return len(self.sequence)

    def get_remaining_frames(self):
        """Return how many frames are left in the current sequence."""
        return max(0, len(self.sequence) - self.index)

    def get_progress(self):
        """Return progress as percentage (0.0 to 1.0)."""
        if len(self.sequence) == 0:
            return 1.0
        return self.index / len(self.sequence)

    def _generate_sequence(self):
        """Generate a new Gaussian torque sequence using current parameters."""
        # Main Gaussian curve with configurable duration
        duration = self.rng.randint(*self.duration_range)
        std = max(duration / self.std_divisor, 1.0)  # Assicura std >= 1.0

        # Gestisci caso peak_torque_range con min==max
        p_min, p_max = self.peak_torque_range
        if p_min == p_max:
            peak_torque = p_min
        else:
            peak_torque = self.rng.randint(p_min, p_max)

        # Main Gaussian curve (normalized to peak_torque)
        seq = _gaussian_window(duration, std)
        seq = seq / np.max(seq) * peak_torque

        # Optional opposite torque pulse (for natural variability)
        if self.rng.random() < self.opposite_probability:
            opposite_peak = self.rng.randint(
                max(1, int(self.peak_torque_range[0] * 0.8)),
                max(1, int(self.peak_torque_range[1] * 0.9))
            )
            opposite_duration = self.rng.randint(
                max(10, int(duration * 0.4)),
                max(10, int(duration * 0.6))
            )
            opposite_std = max(opposite_duration / self.std_divisor, 1.0)
            opposite_seq = -opposite_peak * _gaussian_window(opposite_duration, opposite_std)

            pause_frames = self.rng.randint(
                max(1, int(duration * 0.05)),
                max(1, int(duration * 0.15))
            )
            seq = np.concatenate([seq, np.zeros(pause_frames), opposite_seq])

        # Restituisci come lista di float (conversione a int nel controller)
        return seq.tolist()

    def next_value(self):
        """
        Return the next torque value in the sequence.
        When the sequence ends, returns 0.0 (sequence must be reset manually).

        Returns:
            float: torque value in Nm (saturated to ±eps_max)
        """
        if self.index >= len(self.sequence):
            # Sequenza finita, ritorna 0
            return 0.0

        val = self.sequence[self.index]
        self.index += 1

        # Satura ai limiti EPS (restituisce float, cast a int nel controller)
        return float(np.clip(val, -self.eps_max, self.eps_max))

    def peek_next_value(self):
        """Preview the next value without advancing the index."""
        if self.index >= len(self.sequence):
            return 0.0
        return float(np.clip(self.sequence[self.index], -self.eps_max, self.eps_max))