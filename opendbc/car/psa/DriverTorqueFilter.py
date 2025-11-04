
# class DriverTorqueFilter:
#     """
#     Driver torque filter: deadband -> EMA -> rate-limit  (opz: 2° ordine)
#     """
#     def __init__(self, alpha=0.22, deadband=0.6, rate_limit_per_s=30.0, second_order=True, frequency_hz=100.0):
#         self.alpha = float(alpha)                 # EMA smoothing factor
#         self.db = float(deadband)                # Nm
#         self.dmax = float(rate_limit_per_s) / frequency_hz # Nm/frame
#         self.second_order = bool(second_order)
#         self.y1 = 0.0
#         self.y2 = 0.0  # per 2° ordine

#     @staticmethod
#     def _deadband(x, d):
#         if d <= 0: return x
#         if x >  d: return x - d
#         if x < -d: return x + d
#         return 0.0

#     def _ema(self, y, x):
#         return y + self.alpha * (x - y)

#     def update(self, raw_driver_torque: float) -> float:
#         # 1) deadband
#         x = self._deadband(float(raw_driver_torque), self.db)

#         # 2) EMA (1° ordine)
#         y_new = self._ema(self.y1, x)
#         if self.second_order:
#             # 2b) EMA su EMA (2° ordine)
#             y_new2 = self._ema(self.y2, y_new)
#         else:
#             y_new2 = y_new

#         # 3) rate-limit
#         dy = y_new2 - (self.y2 if self.second_order else self.y1)
#         if   dy >  self.dmax: dy =  self.dmax
#         elif dy < -self.dmax: dy = -self.dmax

#         # aggiorna stati
#         if self.second_order:
#             self.y2 += dy
#             self.y1 = y_new
#             return self.y2
#         else:
#             self.y1 += dy
#             return self.y1

#     def reset(self):
#         self.y1 = 0.0
#         self.y2 = 0.0

class DriverTorqueFilter:
  """Adaptive EMA filter with spike detection for PSA driver torque."""

  __slots__ = ['alpha', 'db', 'dmax', 'second_order', 'spike_threshold',
               'y1', 'y2', '_prev_in']

  def __init__(self, alpha=0.20, deadband=1.0, rate_limit_per_s=40.0,
               second_order=True, frequency_hz=100.0, spike_threshold=3.0):
    self.alpha = alpha
    self.db = deadband
    self.dmax = rate_limit_per_s / frequency_hz
    self.second_order = second_order
    self.spike_threshold = spike_threshold
    self.y1 = 0.0
    self.y2 = 0.0
    self._prev_in = 0.0

  @staticmethod
  def _deadband(x, d):
    if x > d: return x - d
    if x < -d: return x + d
    return 0.0

  def update(self, raw):
    x = self._deadband(raw, self.db)
    is_spike = abs(x - self._prev_in) > self.spike_threshold
    alpha = min(self.alpha * 2.5, 0.45) if is_spike else self.alpha
    dmax = self.dmax * 4.0 if is_spike else self.dmax

    y_new = self.y1 + alpha * (x - self.y1)

    if self.second_order and not is_spike:
      y_target = self.y2 + alpha * (y_new - self.y2)
      dy = max(-dmax, min(dmax, y_target - self.y2))
      self.y2 += dy
      self.y1 = y_new
      result = self.y2
    else:
      dy = max(-dmax, min(dmax, y_new - self.y1))
      self.y1 += dy
      if self.second_order:
        self.y2 = self.y1
      result = self.y1

    self._prev_in = x
    return result

  def reset(self):
    self.y1 = 0.0
    self.y2 = 0.0
    self._prev_in = 0.0
