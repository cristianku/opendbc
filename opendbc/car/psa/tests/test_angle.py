# <TEST_ANGLE_START>
"""Offline angle experiment checks; these do not establish EPS compatibility."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from opendbc.car import Bus, structs
from opendbc.car.psa.carcontroller import CarController
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.psacan import create_lka_steering
from opendbc.car.psa.values import CAR


def decode_lka(data):
  angle = (data[6] << 6) | (data[7] >> 2)
  torque = (data[3] << 3) | (data[4] >> 5)
  return SimpleNamespace(angle=(angle - 16384 if angle & 8192 else angle) / 10,
                         torque=torque - 2048 if torque & 1024 else torque,
                         status=(data[4] >> 2) & 7, factor=data[5] >> 1,
                         drive=(data[0] >> 6) & 1, lxa=data[5] & 1)


class AngleHarness:
  def __init__(self, candidate=CAR.PSA_PEUGEOT_3008, *, angle_enabled=True):
    # Test either implementation independently of the user's selected default.
    with patch('opendbc.car.psa.interface.TEST_ANGLE_ENABLED', angle_enabled):
      self.cp = CarInterface.get_non_essential_params(candidate)
    cp_sp = CarInterface.get_non_essential_params_sp(self.cp, candidate)
    self.controller = CarController({Bus.main: 'psa_aee2010_r3'}, self.cp, cp_sp)
    self.controller.model_sm = None
    self.cs = SimpleNamespace(out=structs.CarState(), eps_active=True, eps_state_lka=3,
                              speed_kph=72, is_dat_dira={}, HS2_DYN_MDD_ETAT_2F6={}, steering={'DRIVER_TORQUE': 0})
    self.cs.out.canValid = True
    self.cs.out.vEgo = self.cs.out.vEgoRaw = 20
    self.cs.out.steeringAngleDeg = 12
    self.cc = structs.CarControl()
    self.cc.latActive = True
    self.cc.actuators.steeringAngleDeg = -20

  def step(self):
    return self.controller.update(self.cc.as_reader(), structs.CarControlSP(), self.cs,
                                  (self.controller.frame + 1) * 10_000_000)

  def lka(self):
    output, messages = self.step()
    frames = [m for m in messages if m[0] == 0x3F2]
    assert len(frames) == 1, f'expected one angle frame, got {frames}'
    return output, decode_lka(frames[0][1]), frames[0]


class TestAngleController(unittest.TestCase):
  def test_one_selector_switches_controller_and_safety_together(self):
    for enabled in (False, True):
      with self.subTest(angle=enabled):
        h = AngleHarness(angle_enabled=enabled)
        h.cc.latActive = False
        _, values, _ = h.lka()
        self.assertEqual(h.cp.steerControlType, structs.CarParams.SteerControlType.angle if enabled
                         else structs.CarParams.SteerControlType.torque)
        self.assertEqual(bool(h.cp.safetyConfigs[0].safetyParam & 2), enabled)
        self.assertEqual(h.controller.test_angle, enabled)
        self.assertEqual((values.drive, values.lxa), (1, 1) if enabled else (0, 0))

  def test_only_3008_selects_experiment(self):
    self.assertEqual(AngleHarness().cp.safetyConfigs[0].safetyParam & 2, 2)
    self.assertEqual(AngleHarness(CAR.PSA_CITROEN_C4_SPACETOURER).cp.steerControlType,
                     structs.CarParams.SteerControlType.torque)
    for alpha_long, expected in ((False, 2), (True, 3)):
      with patch('opendbc.car.psa.interface.TEST_ANGLE_ENABLED', True):
        cp = CarInterface.get_params(CAR.PSA_PEUGEOT_3008, {0: {}, 1: {}, 2: {}}, [], alpha_long, False, False)
      self.assertEqual(cp.safetyConfigs[0].safetyParam, expected)
      self.assertEqual(cp.openpilotLongitudinalControl, alpha_long)

  def test_engagement_starts_at_measured_angle(self):
    output, values, _ = AngleHarness().lka()
    self.assertEqual(values.angle, 12)
    self.assertEqual(output.steeringAngleDeg, 12)
    self.assertEqual((values.torque, values.factor, values.drive, values.lxa), (0, 100, 1, 1))

  def test_cycle_continues_after_eps_ack_without_torque_rearm(self):
    h = AngleHarness()
    for frame in range(1400):
      output, values, message = h.lka()
      self.assertEqual(values.status, (2, 3, 4)[frame // 5 % 3])
      self.assertEqual(values.factor, 100)
      self.assertEqual(values.torque, 0)
      self.assertEqual(message[1][1] & 15, frame % 16)
      self.assertEqual(sum((b >> 4) + (b & 15) for b in message[1]) % 16, 11)
      self.assertEqual((output.torque, output.torqueOutputCan), (0, 0))
      h.cs.out.steeringAngleDeg = values.angle

  def test_release_on_driver_brake_bad_can_fault_and_nonfinite_target(self):
    for field, value in [('steeringPressed', True), ('brakePressed', True), ('canValid', False),
                         ('steerFaultTemporary', True), ('steerFaultPermanent', True)]:
      with self.subTest(field=field):
        h = AngleHarness()
        h.lka()
        setattr(h.cs.out, field, value)
        _, values, _ = h.lka()
        self.assertEqual((values.factor, values.status, values.angle), (0, 0, 12))
    for target in (float('nan'), float('inf'), -float('inf')):
      h = AngleHarness()
      h.cc.actuators.steeringAngleDeg = target
      self.assertEqual(h.lka()[1].factor, 0)

  def test_inactive_follows_measured_angle_and_no_synthetic_hands(self):
    h = AngleHarness()
    for frame in range(100):
      h.cc.latActive = frame < 50
      output, messages = h.step()
      self.assertFalse({0x495, 0x2F5} & {m[0] for m in messages})
      if not h.cc.latActive:
        values = decode_lka(next(m[1] for m in messages if m[0] == 0x3F2))
        self.assertEqual((values.factor, values.status, values.angle), (0, 0, 12))

  def test_missing_eps_ack_stops_attempt_until_disengagement(self):
    h = AngleHarness()
    h.cs.eps_active = False
    h.cs.eps_state_lka = 2
    for _ in range(50):
      self.assertEqual(h.lka()[1].angle, 12)
    self.assertEqual(h.lka()[1].factor, 0)
    h.cs.eps_active = True
    self.assertEqual(h.lka()[1].factor, 0)
    h.cc.latActive = False
    h.lka()
    h.cc.latActive = True
    self.assertEqual(h.lka()[1].factor, 100)

  def test_waiting_for_ack_holds_initial_angle_when_measurement_changes(self):
    h = AngleHarness()
    h.cs.eps_active = False
    h.cs.eps_state_lka = 2
    self.assertEqual(h.lka()[1].angle, 12)
    h.cs.out.steeringAngleDeg = 13
    self.assertEqual(h.lka()[1].angle, 12)

  def test_measured_angle_outside_test_bounds_cannot_activate(self):
    for angle in (-200, 200):
      h = AngleHarness()
      h.cs.out.steeringAngleDeg = angle
      self.assertEqual(h.lka()[1].factor, 0)

  def test_no_request_below_existing_speed_threshold_or_in_passive_mode(self):
    for speed in (0, 10, 14):
      h = AngleHarness()
      h.cs.out.vEgoRaw = speed
      self.assertEqual(h.lka()[1].factor, 0)
    for field in ('dashcamOnly', 'passive'):
      h = AngleHarness()
      setattr(h.cp, field, True)
      self.assertEqual(h.lka()[1].factor, 0)

  def test_nonfinite_measurement_emits_no_angle_frame(self):
    h = AngleHarness()
    h.cs.out.steeringAngleDeg = float('nan')
    _, messages = h.step()
    self.assertNotIn(0x3F2, [m[0] for m in messages])

  def test_rate_limits_and_saturation_in_both_directions(self):
    for sign in (-1, 1):
      h = AngleHarness()
      h.cs.out.steeringAngleDeg = 0
      h.cs.out.vEgo = h.cs.out.vEgoRaw = 25
      h.cc.actuators.steeringAngleDeg = sign * 500
      previous = h.lka()[1].angle
      for _ in range(2500):
        _, values, _ = h.lka()
        self.assertLessEqual(abs(values.angle), 90)
        self.assertLessEqual(abs(values.angle - previous), 0.100001)
        previous = h.cs.out.steeringAngleDeg = values.angle
      self.assertEqual(previous, sign * 90)

  def test_reproduces_recorded_e208_angle_frame(self):
    h = AngleHarness()
    msg = create_lka_steering(h.controller.packer, True, 0, 100, 4, unknown2=0,
                              drive=1, lxa_activation=1, set_angle=-5, counter=7)
    self.assertEqual(msg, (0x3F2, bytes.fromhex('4017000010c9ff38'), 0))

  def test_c4_and_explicit_torque_rollback_keep_legacy_payloads(self):
    for candidate in (CAR.PSA_CITROEN_C4_SPACETOURER, CAR.PSA_PEUGEOT_3008):
      h = AngleHarness(candidate)
      h.cp.steerControlType = structs.CarParams.SteerControlType.torque
      h.cp.safetyConfigs[0].safetyParam &= ~2
      cp_sp = CarInterface.get_non_essential_params_sp(h.cp, candidate)
      h.controller = CarController({Bus.main: 'psa_aee2010_r3'}, h.cp, cp_sp)
      h.cs.eps_active = False
      h.cc.latActive = False
      self.assertEqual(h.lka()[2][1], bytes.fromhex('0000180008000000'))
      h.controller.frame = 5
      h.cc.latActive = True
      self.assertEqual(h.lka()[2][1], bytes.fromhex('000018000c140000'))


if __name__ == '__main__':
  unittest.main()
# <TEST_ANGLE_START_END>
