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

  def test_braking_request_is_amplified_before_existing_limit(self):
    self.assertEqual(LongitudinalParams.BRAKE_ACCEL_GAIN, 1.55)
    self.assertAlmostEqual(self.update_longitudinal(-0.8), -1.24)

  def test_braking_gain_keeps_existing_minus_two_limit(self):
    self.assertAlmostEqual(self.update_longitudinal(-1.8), LongitudinalParams.BRAKE_MIN_ACCEL)
    self.assertEqual(LongitudinalParams.BRAKE_MIN_ACCEL, -2.0)
# [long response] - END


if __name__ == '__main__':
  unittest.main()
