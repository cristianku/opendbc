import math
import random
import unittest

from numpy import interp

from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, DT_CTRL
from opendbc.car.psa.tests.test_longitudinal import LongitudinalHarness
from opendbc.car.psa.values import LongitudinalParams

DESCENTS_DEG = (3.0, 5.0, 8.0)


def descent_pitch(degrees):
  # NED pitch: nose down on a descent is negative.
  return -math.radians(degrees)


def expected_brake(accel):
  # Settled brake command: brake lookup, then the service-brake limit.
  brake_accel = float(interp(accel, LongitudinalParams.BRAKE_ACCEL_BP, LongitudinalParams.BRAKE_ACCEL_V))
  return max(LongitudinalParams.BRAKE_MIN_ACCEL, min(brake_accel, 0.0))


class TestLongitudinalBrakeHysteresis(unittest.TestCase):
  def setUp(self):
    self.h = LongitudinalHarness()
    self.h.controller.radar_active = True
    self.set_pitch(0.0)

  def set_pitch(self, pitch):
    self.h.cc.orientationNED = [0.0, pitch, 0.0]

  def update(self, accel):
    self.h.cc.actuators.accel = accel
    self.h.controller._update_longitudinal(self.h.cc.as_reader(), self.h.cs)
    return self.h.controller.longitudinal_braking, self.h.controller.longitudinal_accel

  def run_sequence(self, accels):
    braking_states = []
    commands = []
    for accel in accels:
      braking, command = self.update(accel)
      braking_states.append(braking)
      commands.append(command)
    return braking_states, commands

  def count_transitions(self, braking_states):
    transitions = 0
    for i in range(1, len(braking_states)):
      if braking_states[i] != braking_states[i - 1]:
        transitions += 1
    return transitions

  # --- level road ---------------------------------------------------------

  def test_level_road_entry_and_exit_thresholds(self):
    self.assertFalse(self.update(-0.45)[0])
    self.assertTrue(self.update(-0.55)[0])
    for accel in (-0.3, -0.05, 0.0):
      with self.subTest(accel=accel):
        self.assertTrue(self.update(accel)[0])
    self.assertFalse(self.update(0.05)[0])

  def test_level_road_noise_around_zero_never_enters_braking(self):
    rng = random.Random(1)
    accels = [rng.uniform(-0.2, 0.2) for _ in range(2000)]
    braking_states, _ = self.run_sequence(accels)
    self.assertEqual(self.count_transitions(braking_states), 0)
    self.assertFalse(braking_states[-1])

  def test_level_road_light_braking_noise_stays_in_brake_mode(self):
    self.run_sequence([-0.8] * 100)
    rng = random.Random(2)
    accels = [-0.05 + rng.uniform(-0.04, 0.04) for _ in range(2000)]
    braking_states, commands = self.run_sequence(accels)
    self.assertEqual(self.count_transitions(braking_states), 0)
    self.assertTrue(braking_states[-1])
    for command in commands:
      self.assertLessEqual(command, 0.0)

  # --- descents -------------------------------------------------------------

  def test_descent_zero_request_enters_braking_with_hold_command(self):
    for degrees in DESCENTS_DEG:
      with self.subTest(degrees=degrees):
        self.setUp()
        self.set_pitch(descent_pitch(degrees))
        braking, command = self.update(0.0)
        self.assertTrue(braking)
        # Brake ECU regulates vehicle deceleration: 0 means hold speed, no grade term.
        self.assertEqual(command, 0.0)

  def test_descent_small_request_oscillations_do_not_toggle(self):
    for degrees in DESCENTS_DEG:
      for amplitude in (0.05, 0.1, 0.15):
        with self.subTest(degrees=degrees, amplitude=amplitude):
          self.setUp()
          self.set_pitch(descent_pitch(degrees))
          rng = random.Random(3)
          accels = []
          for i in range(1500):
            t = i * DT_CTRL
            accels.append(amplitude * math.sin(2 * math.pi * 1.0 * t) + rng.uniform(-0.02, 0.02))
          braking_states, commands = self.run_sequence(accels)
          first = braking_states.index(True)
          self.assertLess(first, 10)
          self.assertEqual(self.count_transitions(braking_states[first:]), 0)
          for command in commands[first:]:
            self.assertLessEqual(command, 0.0)
            self.assertGreaterEqual(command, -amplitude - 0.03)

  def test_descent_old_failure_mode_is_gone(self):
    # Old code: entry at accel <= 0 (grade) and exit at accel > 0 -> one toggle per zero crossing.
    self.set_pitch(descent_pitch(5.0))
    accels = []
    for i in range(1000):
      if (i // 25) % 2 == 0:
        accels.append(0.03)
      else:
        accels.append(-0.03)
    braking_states, _ = self.run_sequence(accels)
    first = braking_states.index(True)
    self.assertEqual(self.count_transitions(braking_states[first:]), 0)

  def test_descent_progressive_braking_is_monotonic(self):
    for degrees in DESCENTS_DEG:
      with self.subTest(degrees=degrees):
        self.setUp()
        self.set_pitch(descent_pitch(degrees))
        accels = []
        for i in range(300):
          accels.append(-1.5 * i / 299)
        accels += [-1.5] * 200
        braking_states, commands = self.run_sequence(accels)
        self.assertTrue(all(braking_states))
        for i in range(1, len(commands)):
          self.assertLessEqual(commands[i], commands[i - 1] + 1e-9)
        self.assertAlmostEqual(commands[-1], expected_brake(-1.5), delta=0.01)

  def test_descent_positive_request_leaves_brakes_when_torque_can_follow(self):
    # 5 deg: +0.4 -> equivalent -0.455 > enter threshold -> torque path.
    self.set_pitch(descent_pitch(5.0))
    self.run_sequence([0.0] * 50)
    braking_states, commands = self.run_sequence([0.4] * 50)
    self.assertFalse(braking_states[-1])
    self.assertLessEqual(self.count_transitions(braking_states), 1)
    self.assertLess(braking_states.index(False), 10)  # positive jerk ramp only
    self.assertAlmostEqual(commands[-1], 0.4, delta=0.001)

  def test_steep_descent_positive_request_keeps_hold_brake(self):
    # 8 deg: +0.4 -> equivalent -0.965, beyond what the torque map can hold: keep the
    # brake at 0 (hold speed) until the request reaches equivalent > enter threshold.
    self.set_pitch(descent_pitch(8.0))
    self.run_sequence([0.0] * 50)
    braking_states, commands = self.run_sequence([0.4] * 100)
    self.assertTrue(all(braking_states))
    self.assertEqual(commands[-1], 0.0)
    grade = ACCELERATION_DUE_TO_GRAVITY * math.sin(descent_pitch(8.0))
    exit_accel = LongitudinalParams.BRAKE_ENTER_ACCEL - grade + 0.05
    braking_states, _ = self.run_sequence([exit_accel] * 100)
    self.assertFalse(braking_states[-1])

  def test_descent_reentry_after_positive_exit_needs_non_positive_request(self):
    self.set_pitch(descent_pitch(5.0))
    self.run_sequence([0.0] * 50 + [0.4] * 50)
    self.assertFalse(self.h.controller.longitudinal_braking)
    braking_states, _ = self.run_sequence([0.1] * 100)
    self.assertFalse(any(braking_states))
    braking_states, commands = self.run_sequence([0.0] * 10)
    self.assertTrue(braking_states[0])
    self.assertEqual(commands[0], 0.0)

  # --- uphill and grade -----------------------------------------------------

  def test_uphill_uses_torque_until_grade_compensated_request_needs_brakes(self):
    self.set_pitch(math.radians(3.0))
    braking, _ = self.update(-0.8)
    self.assertFalse(braking)
    self.assertGreater(self.h.controller.longitudinal_wheel_torque, LongitudinalParams.INACTIVE_TORQUE)
    braking_states, commands = self.run_sequence([-1.2] * 300)
    self.assertTrue(braking_states[-1])
    self.assertAlmostEqual(commands[-1], expected_brake(-1.2), delta=0.001)

  def test_brake_command_has_no_grade_term(self):
    for degrees in (8.0, 5.0, 3.0, 0.0, -3.0):  # descent degrees; -3 = 3 deg uphill
      with self.subTest(degrees=degrees):
        self.setUp()
        self.set_pitch(descent_pitch(degrees))
        braking_states, commands = self.run_sequence([-1.6] * 300)
        self.assertTrue(braking_states[-1])
        self.assertAlmostEqual(commands[-1], expected_brake(-1.6), delta=0.001)

  def test_single_cycle_pitch_spike_does_not_change_mode(self):
    # Spikes of ~0.04 rad in one 10 ms step are present in recorded livePose pitch.
    self.set_pitch(descent_pitch(5.0))
    self.run_sequence([0.0] * 200)
    self.set_pitch(descent_pitch(5.0) + 0.6)
    self.assertTrue(self.update(0.0)[0])
    self.set_pitch(descent_pitch(5.0))
    self.assertTrue(self.update(0.0)[0])

    self.setUp()
    self.run_sequence([-0.3] * 200)
    self.set_pitch(-0.6)
    self.assertFalse(self.update(-0.3)[0])

  def test_invalid_pitch_while_braking_keeps_brake_with_last_grade(self):
    self.set_pitch(descent_pitch(5.0))
    self.run_sequence([-0.4] * 200)
    self.set_pitch(math.nan)
    braking, command = self.update(-0.4)
    self.assertTrue(braking)
    self.assertTrue(self.h.controller.longitudinal_active)
    self.assertAlmostEqual(command, expected_brake(-0.4), delta=0.01)
    # Leaving the brakes would need the torque map, which needs a valid grade.
    braking, command = self.update(1.0)
    self.assertFalse(self.h.controller.longitudinal_active)
    self.assertEqual(command, 0.0)

  # --- release, disengagement, driver ------------------------------------------

  def test_level_road_immediate_release_has_no_brake_tail(self):
    self.run_sequence([-1.0] * 200)
    braking, command = self.update(0.5)
    self.assertFalse(braking)
    self.assertGreater(command, 0.0)
    self.assertEqual(self.h.controller.brake_accel_filter.x, 0.0)

  def test_weaker_request_reduces_brake_immediately(self):
    self.set_pitch(descent_pitch(5.0))
    self.run_sequence([-1.0] * 200)
    braking, command = self.update(-0.2)
    self.assertTrue(braking)
    self.assertAlmostEqual(command, expected_brake(-0.2))

  def test_driver_and_disengagement_clear_brake_mode_on_descent(self):
    gates = (
      ('gasPressed', 'out', True),
      ('brakePressed', 'out', True),
      ('longActive', 'cc', False),
      ('enabled', 'cc', False),
      ('enabled', 'cruise', False),
    )
    for field, owner, value in gates:
      with self.subTest(field=field):
        self.setUp()
        self.set_pitch(descent_pitch(5.0))
        self.run_sequence([-1.0] * 200)
        target = {'out': self.h.cs.out, 'cc': self.h.cc, 'cruise': self.h.cs.out.cruiseState}[owner]
        previous = getattr(target, field)
        setattr(target, field, value)
        braking, command = self.update(-1.0)
        self.assertFalse(braking)
        self.assertFalse(self.h.controller.longitudinal_active)
        self.assertEqual(command, 0.0)
        setattr(target, field, previous)
        # Re-entry starts from a reset filter: no stale -1.0 jump.
        braking, command = self.update(-1.0)
        self.assertTrue(braking)
        self.assertGreater(command, 0.5 * expected_brake(-1.0))

  # --- limits and CAN encoding ----------------------------------------------

  def test_saturation_never_exceeds_safety_brake_limit(self):
    self.set_pitch(descent_pitch(8.0))
    _, commands = self.run_sequence([-10.0] * 500)
    for command in commands:
      self.assertGreaterEqual(command, LongitudinalParams.BRAKE_MIN_ACCEL)
      self.assertLessEqual(command, 0.0)
    self.assertAlmostEqual(commands[-1], LongitudinalParams.BRAKE_MIN_ACCEL, delta=0.0001)

  def test_can_encoding_on_descent_stays_inside_safety_brake_window(self):
    h = LongitudinalHarness()
    stock = [h.packer.make_can_msg(name, 1, {}) for name in ('HS2_DYN_MDD_ETAT_2F6', 'HS2_DAT_ARTIV_V2_4F6')]
    h.controller.process_radar_can([(10_000_000_000, stock)])
    h.activate()
    h.cc.orientationNED = [0.0, descent_pitch(5.0), 0.0]
    for accel in (0.0, -0.02, -0.03, -0.26, -1.0, -2.0, -3.0, 0.1):
      h.cc.actuators.accel = accel
      for _ in range(150):
        _, values = h.emission()
      b6 = values[0x2B6]
      with self.subTest(accel=accel):
        self.assertEqual(b6['MDD_DECEL_CONTROL_REQ'], 1)
        self.assertEqual(b6['WHEEL_TORQUE_REQUEST'], 0)
        raw = round((b6['MDD_DESIRED_DECELERATION'] + 10.65) / 0.05)
        self.assertGreaterEqual(raw, 173)  # psa.h braking window: -2.0 .. 0.0 m/s^2
        self.assertLessEqual(raw, 213)
        expected = expected_brake(accel)
        self.assertLessEqual(abs(b6['MDD_DESIRED_DECELERATION'] - expected), 0.025 + 1e-6)


if __name__ == '__main__':
  unittest.main()
