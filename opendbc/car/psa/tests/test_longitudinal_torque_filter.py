import unittest

from opendbc.car import DT_CTRL
from opendbc.car.common.filter_simple import FirstOrderFilter
from opendbc.car.psa.carcontroller import CarController
from opendbc.car.psa.values import LongitudinalParams


# [torque filter] - START
class TestLongitudinalTorqueFilter(unittest.TestCase):
  def setUp(self):
    self.assertEqual(LongitudinalParams.TORQUE_FILTER_RC, 0.20)
    self.controller = object.__new__(CarController)
    self.controller.wheel_torque_filter = FirstOrderFilter(0., LongitudinalParams.TORQUE_FILTER_RC, DT_CTRL)
    self.controller.potential_torque_filter = FirstOrderFilter(0., LongitudinalParams.TORQUE_FILTER_RC, DT_CTRL)

  def test_positive_step_is_low_pass_filtered(self):
    wheel, potential = self.controller._filter_longitudinal_torque(300., 280.)
    self.assertAlmostEqual(wheel, 14.2857142857)
    self.assertAlmostEqual(potential, 13.3333333333)

    next_wheel, next_potential = self.controller._filter_longitudinal_torque(300., 280.)
    self.assertGreater(next_wheel, wheel)
    self.assertLess(next_wheel, 300.)
    self.assertGreater(next_potential, potential)
    self.assertLess(next_potential, 280.)

  def test_torque_reduction_is_immediate(self):
    self.controller.wheel_torque_filter.x = 300.
    self.controller.potential_torque_filter.x = 280.

    wheel, potential = self.controller._filter_longitudinal_torque(100., 80.)

    self.assertEqual(wheel, 100.)
    self.assertEqual(potential, 80.)
    self.assertEqual(self.controller.wheel_torque_filter.x, 100.)
    self.assertEqual(self.controller.potential_torque_filter.x, 80.)

  def test_reset_starts_next_positive_step_from_zero(self):
    self.controller.wheel_torque_filter.x = 250.
    self.controller.potential_torque_filter.x = 220.

    self.controller._reset_longitudinal_torque_filters()

    self.assertEqual(self.controller.wheel_torque_filter.x, 0.)
    self.assertEqual(self.controller.potential_torque_filter.x, 0.)
    wheel, potential = self.controller._filter_longitudinal_torque(300., 280.)
    self.assertAlmostEqual(wheel, 14.2857142857)
    self.assertAlmostEqual(potential, 13.3333333333)
# [torque filter] - END


if __name__ == '__main__':
  unittest.main()
