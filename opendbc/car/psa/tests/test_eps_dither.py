import random
import unittest

from opendbc.car import DT_CTRL
from opendbc.car.psa.tests.test_lka_100hz import ReleaseHarness

EXPECTED_BURST = [0, 0, 0, 1, 0, -2, -3, -3, 0, 4, 6, 4, 0, -4, -6, -4, 0, 3, 3, 2, 0, -1, 0, 0]


class TestEpsDither(unittest.TestCase):
  def harness(self, amplitude=6):
    h = ReleaseHarness(self)
    h.controller.params.EPS_DITHER_AMPLITUDE = amplitude
    h.controller.eps_dither_rng = random.Random(0)
    self.values = []
    original = h.controller._eps_dither

    def recorder():
      v = original()
      self.values.append((h.controller.frame, v))
      return v

    h.controller._eps_dither = recorder
    return h

  def burst_values(self):
    out = []
    for _, v in self.values:
      out.append(v)
    return out

  def test_waveform(self):
    h = ReleaseHarness(self)
    c = h.controller
    c.params.EPS_DITHER_AMPLITUDE = 6
    c.eps_dither_sign = 1
    c.eps_dither_start_frame = 0
    values = []
    for k in range(30):
      c.frame = k * 5  # sampled every 50 ms
      values.append(c._eps_dither())
    self.assertEqual(values[:24], EXPECTED_BURST)
    self.assertEqual(values[24:], [0] * 6)
    self.assertEqual(c.eps_dither_start_frame, -1)
    self.assertEqual(sum(values), 0)
    for i in range(1, len(values)):
      self.assertLessEqual(abs(values[i] - values[i - 1]), 4)

  def test_schedule_window_and_alternating_sign(self):
    h = ReleaseHarness(self)
    c = h.controller
    c.params.EPS_DITHER_AMPLITUDE = 6
    c.eps_dither_rng = random.Random(0)
    burst_time = c.params.EPS_DITHER_WAVES / c.params.EPS_DITHER_FREQUENCY
    latest = c.eps_rearm_frames * DT_CTRL - c.params.EPS_DITHER_END_MARGIN - burst_time
    sign = c.eps_dither_sign
    for i in range(200):
      c.frame = 1000 + i * 2000
      c._start_eps_active_cycle()
      delay = (c.eps_dither_start_frame - c.frame) * DT_CTRL
      self.assertGreaterEqual(delay, c.params.EPS_DITHER_START_MARGIN - 0.03)
      self.assertLessEqual(delay, latest + 0.03)
      self.assertEqual((c.eps_dither_start_frame - c.frame) % c.params.STEER_STEP, 0)
      self.assertEqual(c.eps_dither_sign, -sign)
      sign = c.eps_dither_sign

  def test_amplitude_zero_means_no_burst(self):
    h = self.harness(amplitude=0)
    for _ in range(1000):
      h.step()
    self.assertEqual(h.controller.eps_dither_start_frame, -1)
    for v in self.burst_values():
      self.assertEqual(v, 0)

  def test_full_burst_through_safety(self):
    # Steady request: the whole burst goes out, every frame accepted by the panda safety.
    h = self.harness()
    for _ in range(1000):
      h.step()
    values = self.burst_values()
    nonzero = 0
    for v in values:
      if v != 0:
        nonzero += 1
    self.assertGreater(nonzero, 10)
    self.assertEqual(sum(values), 0)
    self.assertEqual(h.controller.eps_dither_start_frame, -1)

  def test_clipping_drops_the_burst(self):
    h = self.harness()
    for _ in range(150):
      h.step()
    start = h.controller.eps_dither_start_frame
    self.assertGreater(start, 0)
    while h.controller.frame < start + 30:
      h.step()
    # Big jump in the request: the base ramps at the rate limit, the dither would be clipped.
    h.cc.actuators.torque = -1.0
    for _ in range(10):
      h.step()
    self.assertEqual(h.controller.eps_dither_start_frame, -1)
    count = len(self.values)
    for _ in range(200):
      h.step()
    for _, v in self.values[count:]:
      self.assertEqual(v, 0)

  def test_override_ends_the_burst_without_resume(self):
    h = self.harness()
    for _ in range(150):
      h.step()
    start = h.controller.eps_dither_start_frame
    while h.controller.frame < start + 30:
      h.step()
    h.cs.out.steeringPressed = True
    for _ in range(10):
      h.step()
    h.cs.out.steeringPressed = False
    self.assertEqual(h.controller.eps_dither_start_frame, -1)
    count = len(self.values)
    for _ in range(200):
      h.step()
    for _, v in self.values[count:]:
      self.assertEqual(v, 0)


if __name__ == "__main__":
  unittest.main()
