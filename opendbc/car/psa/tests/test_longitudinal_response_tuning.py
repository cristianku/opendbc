import unittest

from opendbc.car import DT_CTRL
from opendbc.car.psa.tests.test_longitudinal import LongitudinalHarness
from opendbc.car.psa.values import LongitudinalParams


# [long response] - START
class TestLongitudinalResponseTuning(unittest.TestCase):
  def setUp(self):
    self.h = LongitudinalHarness()
    self.h.controller.longitudinal_enabled = True
    self.h.controller.radar_active = True
    self.h.controller.longitudinal_accel_limited = 0.0
    self.h.controller.longitudinal_braking = False

  def update_longitudinal(self, accel):
    self.h.cc.actuators.accel = accel
    self.h.controller._update_longitudinal(self.h.cc.as_reader(), self.h.cs)
    return self.h.controller.longitudinal_accel

  def test_positive_acceleration_uses_faster_jerk_limit(self):
    self.assertEqual(LongitudinalParams.POSITIVE_JERK_MAX, 3.5)
    self.assertAlmostEqual(self.update_longitudinal(2.0), 3.5 * DT_CTRL)

  # def test_braking_request_is_amplified_before_existing_limit(self):
  #   self.assertEqual(LongitudinalParams.BRAKE_ACCEL_GAIN, 1.55)
  #   for _ in range(300):
  #     applied = self.update_longitudinal(-0.8)
  #   self.assertAlmostEqual(applied, -1.24, delta=0.0001)
  #
  # def test_braking_gain_keeps_existing_minus_two_limit(self):
  #   for _ in range(300):
  #     applied = self.update_longitudinal(-1.8)
  #   self.assertAlmostEqual(applied, LongitudinalParams.BRAKE_MIN_ACCEL, delta=0.0001)
  #   self.assertEqual(LongitudinalParams.BRAKE_MIN_ACCEL, -2.0)
  def test_brake_gain_is_removed(self):
    self.assertFalse(hasattr(LongitudinalParams, 'BRAKE_ACCEL_GAIN'))

  def test_brake_lookup_is_ten_percent_above_request(self):
    bp = LongitudinalParams.BRAKE_ACCEL_BP
    v = LongitudinalParams.BRAKE_ACCEL_V
    self.assertEqual(len(bp), len(v))
    for i in range(len(bp) - 1):
      self.assertLess(bp[i], bp[i + 1])
      self.assertLess(v[i], v[i + 1])
    self.assertEqual(v[0], LongitudinalParams.BRAKE_MIN_ACCEL)
    for i in range(1, len(bp)):
      self.assertAlmostEqual(v[i], bp[i] * 1.1, delta=0.0001)
    self.assertEqual(bp[0], LongitudinalParams.BRAKE_MIN_ACCEL)
    self.assertEqual(bp[-1], 0.0)
    self.assertEqual(LongitudinalParams.BRAKE_MIN_ACCEL, -2.0)

  def test_settled_brake_command_follows_lookup_and_limit(self):
    for requested, expected in ((-0.1, -0.11), (-0.8, -0.88), (-1.8, -1.86), (-2.5, -2.0), (-10.0, -2.0)):
      with self.subTest(requested=requested):
        self.setUp()
        self.update_longitudinal(-0.8)  # level road: enter below -0.5, then hold light braking
        for _ in range(300):
          applied = self.update_longitudinal(requested)
        self.assertTrue(self.h.controller.longitudinal_braking)
        self.assertAlmostEqual(applied, expected, delta=0.0001)
# [long response] - END


if __name__ == '__main__':
  unittest.main()
