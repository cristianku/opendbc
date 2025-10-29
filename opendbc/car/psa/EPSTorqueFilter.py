class EPSTorqueFilter:
    """
    Filtra e decima il driver torque per simulare il comportamento EPS Peugeot.

    Sistema PSA:
    - Input: driver_torque raw @ 100Hz (noisy)
    - Output: eps_torque filtered @ 10Hz (smoothed, quantized to 0.25 Nm)

    Implementazione:
    - Low-pass IIR filter per smoothing
    - Decimazione 10:1 (100Hz → 10Hz)
    """

    def __init__(self, alpha=0.15):
        """
        Args:
            alpha: Coefficiente del filtro IIR (0-1).
                   Valori più bassi = più smooth, più lag
                   Valori più alti = più reattivo, meno smooth
                   Default 0.15 è un buon compromesso per PSA
        """
        self.filtered = 0.0
        self.frame_count = 0
        self.alpha = alpha

    @staticmethod
    def convert_driver_torque_to_eps(driver_torque: float | int) -> float:
        """
        Converte driver torque (raw) a EPS torque (smoothed).

        PSA system:
        - Driver torque: 100Hz, noisy signal
        - EPS torque: 10Hz, filtered and quantized to 0.25 Nm steps

        Args:
            driver_torque: Raw driver torque value (0-30 from generator)

        Returns:
            EPS torque in Nm, quantized to 0.25 steps
        """
        # Scaling factor: empirically derived from PSA 3008 observations
        # driver_torque range ~0-30 → eps_torque range ~0-3.0 Nm
        SCALE_FACTOR = 2.4

        # Convert and apply intermediate rounding (simulates internal quantization)
        scaled = driver_torque / SCALE_FACTOR

        # Quantize to 0.25 Nm steps (EPS resolution)
        quantized = round(scaled * 4) / 4  # Round to nearest 0.25

        return round(quantized, 2)

    def update(self, driver_torque: float) -> float | None:
        """
        Aggiorna il filtro con un nuovo campione.

        Args:
            driver_torque: Valore raw del driver torque (chiamato @ 100Hz)

        Returns:
            eps_torque filtrato ogni 10 frame, None negli altri frame
        """
        # Low-pass IIR filter: y[n] = α * x[n] + (1-α) * y[n-1]
        self.filtered = self.alpha * driver_torque + (1 - self.alpha) * self.filtered

        # Decimazione: restituisce valore solo ogni 10 frame (100Hz → 10Hz)
        self.frame_count += 1
        if self.frame_count >= 10:
            self.frame_count = 0
            # Converte e quantizza a 0.25 Nm steps
            return self.convert_driver_torque_to_eps(self.filtered)
        return None