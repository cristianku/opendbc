# EPSTorqueFilter.py
class EPSTorqueConverter:
    """
    Convert driver torque values to EPS torque format matching PSA CAN protocol.
    Applies scaling, offset correction, and quantization to match ECU behavior.
    """

    def __init__(self, scale_factor=10.545, offset=0.135, quantization=0.25):
        """
        Args:
            scale_factor: Conversion ratio between driver and EPS torque scales
            offset: Zero-point offset correction (Nm)
            quantization: Output resolution step size (Nm)
        """
        self.scale_factor = float(scale_factor)
        self.offset = float(offset)
        self.quantization = float(quantization)

        # Hysteresis state for hold detection (0=released, 1=holding)
        self.hold_state = 0

    def convert_driver_torque_to_eps(self, driver_torque: float) -> float:
        """
        Convert driver torque to EPS torque with quantization.

        Args:
            driver_torque: Raw driver torque in Nm

        Returns:
            EPS torque value quantized to protocol resolution
        """
        # Apply scale and offset
        scaled = (driver_torque + self.offset) / self.scale_factor

        # Quantize to EPS resolution (default 0.25 Nm steps)
        quantized = round(scaled / self.quantization) * self.quantization

        return round(quantized, 2)

    def convert_driver_to_hold(self, driver_torque: float) -> int:
        """
        Detect steering hold state using hysteresis thresholds.

        Upper threshold: 7 Nm (engage) - increased to reduce false positives
        Lower threshold: 3 Nm (release) - decreased to reduce false negatives
        """
        t = abs(driver_torque)

        # Upper threshold: engage when torque exceeds 7 Nm
        if self.hold_state == 0 and t >= 7:
            self.hold_state = 1
        # Lower threshold: release when torque drops below 3 Nm
        elif self.hold_state == 1 and t <= 3:
            self.hold_state = 0

        return self.hold_state

        def reset(self):
            """Reset internal state (useful when restarting or after errors)."""
            self.hold_state = 0

    def reset(self):
        """Reset internal state (useful when restarting or after errors)."""
        self.hold_state = 0




class DriverTorqueFilter:
    """
    20 Hz: deadband -> EMA -> rate-limit  (opz: 2° ordine)
    """
    def __init__(self, alpha=0.22, deadband=0.6, rate_limit_per_s=30.0, second_order=True):
        self.alpha = float(alpha)                 # 0.08–0.15 consigliato @20Hz
        self.db = float(deadband)                # Nm
        self.dmax = float(rate_limit_per_s) / 20 # Nm/frame @20Hz
        self.second_order = bool(second_order)
        self.y1 = 0.0
        self.y2 = 0.0  # per 2° ordine

    @staticmethod
    def _deadband(x, d):
        if d <= 0: return x
        if x >  d: return x - d
        if x < -d: return x + d
        return 0.0

    def _ema(self, y, x):
        return y + self.alpha * (x - y)

    def update(self, raw_driver_torque: float) -> float:
        # 1) deadband
        x = self._deadband(float(raw_driver_torque), self.db)

        # 2) EMA (1° ordine)
        y_new = self._ema(self.y1, x)
        if self.second_order:
            # 2b) EMA su EMA (2° ordine)
            y_new2 = self._ema(self.y2, y_new)
        else:
            y_new2 = y_new

        # 3) rate-limit
        dy = y_new2 - (self.y2 if self.second_order else self.y1)
        if   dy >  self.dmax: dy =  self.dmax
        elif dy < -self.dmax: dy = -self.dmax

        # aggiorna stati
        if self.second_order:
            self.y2 += dy
            self.y1 = y_new
            return self.y2
        else:
            self.y1 += dy
            return self.y1

    def reset(self):
        self.y1 = 0.0
        self.y2 = 0.0
