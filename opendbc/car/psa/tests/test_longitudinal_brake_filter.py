# [brake filter] - START
import math
import unittest

from opendbc.car.psa.tests.test_longitudinal import LongitudinalHarness
# [brake filter] - END


# [brake filter] - START
class TestLongitudinalBrakeFilter(unittest.TestCase):
  def setUp(self):
    self.h = LongitudinalHarness()
    self.h.controller.radar_active = True

  def update(self, accel, cycles=1):
    self.h.cc.actuators.accel = accel
    samples = []
    for _ in range(cycles):
      self.h.controller._update_longitudinal(self.h.cc.as_reader(), self.h.cs)
      samples.append(self.h.controller.longitudinal_accel)
    return samples

  def test_brake_step_builds_monotonically_to_ninety_percent_in_half_second(self):
    # Immediate braking would fail the first-cycle and half-second bounds.
    samples = self.update(-0.8, 200)
    self.assertGreater(samples[0], -0.07)
    self.assertLess(samples[0], -0.04)
    self.assertTrue(all(-1.24 <= following <= previous for previous, following in zip(samples, samples[1:])))
    self.assertAlmostEqual(samples[46] / -1.24, 0.90, delta=0.005)
    self.assertAlmostEqual(samples[-1], -1.24, delta=0.0001)

  def test_stronger_brake_demand_is_smoothed_after_clamping(self):
    self.update(-0.6, 300)
    previous = self.h.controller.longitudinal_accel
    samples = self.update(-10.0, 200)
    self.assertLess(samples[0], previous)
    self.assertGreater(samples[0], previous - 0.1)
    self.assertTrue(all(-2.0 <= value <= 0.0 for value in samples))
    self.assertAlmostEqual(samples[-1], -2.0, delta=0.0001)

  def test_weaker_braking_and_zero_release_without_filter_tail(self):
    self.update(-0.8, 200)
    self.assertAlmostEqual(self.update(-0.1)[0], -0.155)
    self.assertEqual(self.update(0.0)[0], 0.0)
    self.assertTrue(self.h.controller.longitudinal_braking)

  def test_gas_override_clears_brake_and_resume_starts_from_zero(self):
    first = self.update(-0.8)[0]
    self.update(-0.8, 200)
    self.h.cs.out.gasPressed = True
    self.assertEqual(self.update(-0.8)[0], 0.0)
    self.assertTrue(self.h.controller.acc_on_hold)
    self.h.cs.out.gasPressed = False
    self.assertAlmostEqual(self.update(-0.8)[0], first)

  def test_inactive_and_invalid_inputs_clear_stale_braking(self):
    gates = (
      ('brakePressed', 'out', True),
      ('canValid', 'out', False),
      ('enabled', 'cc', False),
      ('longActive', 'cc', False),
      ('enabled', 'cruise', False),
      ('radar_active', 'controller', False),
      ('longitudinal_enabled', 'controller', False),
    )
    for field, owner, value in gates:
      with self.subTest(field=field, owner=owner):
        self.setUp()
        first = self.update(-0.8)[0]
        self.update(-0.8, 200)
        target = {'out': self.h.cs.out, 'cc': self.h.cc, 'cruise': self.h.cs.out.cruiseState,
                  'controller': self.h.controller}[owner]
        previous = getattr(target, field)
        setattr(target, field, value)
        self.assertEqual(self.update(-0.8)[0], 0.0)
        self.assertFalse(self.h.controller.longitudinal_active)
        setattr(target, field, previous)
        self.assertAlmostEqual(self.update(-0.8)[0], first)
    for invalid in (math.nan, math.inf, -math.inf):
      with self.subTest(invalid=invalid):
        self.setUp()
        first = self.update(-0.8)[0]
        self.update(-0.8, 200)
        self.assertEqual(self.update(invalid)[0], 0.0)
        self.assertAlmostEqual(self.update(-0.8)[0], first)

  def test_positive_request_releases_brake_and_next_entry_is_smoothed(self):
    first = self.update(-0.8)[0]
    self.update(-0.8, 200)
    self.assertGreater(self.update(0.2)[0], 0.0)
    self.assertFalse(self.h.controller.longitudinal_braking)
    self.assertAlmostEqual(self.update(-0.8)[0], first)

  def test_invalid_pitch_on_torque_path_clears_brake_filter(self):
    first = self.update(-0.8)[0]
    self.update(-0.8, 200)
    self.h.cc.orientationNED = [0.0, math.nan, 0.0]
    self.assertEqual(self.update(0.2)[0], 0.0)
    self.assertFalse(self.h.controller.longitudinal_active)
    self.h.cc.orientationNED = [0.0, 0.0, 0.0]
    self.assertAlmostEqual(self.update(-0.8)[0], first)

  def test_can_braking_flags_and_limits_hold_during_filtered_ramp(self):
    self.h = LongitudinalHarness()
    # Feed the stock phase messages so real TX receipts keep the session alive.
    stock = [self.h.packer.make_can_msg(name, 1, {}) for name in ('HS2_DYN_MDD_ETAT_2F6', 'HS2_DAT_ARTIV_V2_4F6')]
    self.h.controller.process_radar_can([(10_000_000_000, stock)])
    self.h.activate()
    self.h.cc.actuators.accel = -10.0
    samples = []
    for _ in range(100):
      _, values = self.h.emission()
      b6, f6 = values[0x2B6], values[0x2F6]
      samples.append(b6['MDD_DESIRED_DECELERATION'])
      self.assertEqual(b6['MDD_DECEL_CONTROL_REQ'], 1)
      self.assertEqual(f6['MDD_DECEL_CONTROL_REQ'], 1)
      self.assertEqual(b6['WHEEL_TORQUE_REQUEST'], 0)
      self.assertEqual(b6['GMP_WHEEL_TORQUE'], -4000)
      self.assertGreaterEqual(samples[-1], -2.0)
      self.assertLessEqual(samples[-1], 0.0)
    self.assertGreater(samples[0], -0.25)
    self.assertEqual(samples[-1], -2.0)
# [brake filter] - END


# [brake filter] - START
if __name__ == '__main__':
  unittest.main()
# [brake filter] - END
