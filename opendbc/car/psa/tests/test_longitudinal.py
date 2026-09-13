# [psa longitudinal] - START
import inspect
import json
import math
import unittest
from collections import Counter
from pathlib import Path
from types import SimpleNamespace

from opendbc.can.packer import CANPacker
from opendbc.can.parser import get_raw_value
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.psa import psacan
from opendbc.car.psa.carcontroller import CarController
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.values import CAR, PSA_LONG_CONTROL

RADAR_IDS = {0x2B6, 0x2F6, 0x4F6, 0x796}


class LongitudinalHarness:
  def __init__(self, *, dashcam=False, passive=False, experimental=True, safety_flag=True):
    cp = CarInterface.get_params(CAR.PSA_PEUGEOT_3008, {0: {}, 1: {}, 2: {}}, [], experimental, False, False)
    cp.openpilotLongitudinalControl = experimental
    cp.dashcamOnly = dashcam
    cp.passive = passive
    cp.safetyConfigs[0].safetyParam = PSA_LONG_CONTROL if experimental and safety_flag else 0
    cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.PSA_PEUGEOT_3008)
    self.controller = CarController({Bus.main: 'psa_aee2010_r3'}, cp, cp_sp)
    self.cs = SimpleNamespace(eps_active=False, out=structs.CarState())
    self.cs.out.standstill = True
    self.cs.out.canValid = True
    self.cs.out.cruiseState.enabled = True
    self.cs.out.cruiseState.speed = 20.0
    self.cc = structs.CarControl()
    self.cc.enabled = True
    self.cc.longActive = True
    self.cc.actuators.accel = 0.5
    self.frame = 1000
    self.previous = []
    self.packer = CANPacker('psa_aee2010_r3')

  def activate(self):
    radar = self.controller
    radar.process_radar_can([(10_000_000_000, [(0x2B6, bytes.fromhex('fe0000020000030a'), 1)])])
    self.step()
    radar.process_radar_can([(10_067_000_000, [(0x696, bytes.fromhex('06500200c80014'), 1)])])
    self.frame = 1011

  def step(self):
    now = self.frame * 10_000_000
    rx = [(0x212, bytes(8), 1)] + [(a, d, 129) for a, d, _ in self.previous if a in RADAR_IDS]
    if any(a == 0x6B6 and d == b'\x02\x3e\x00' for a, d, _ in self.previous):
      rx.append((0x696, b'\x02\x7e\x00', 1))
    self.controller.process_radar_can([(now, rx)])
    self.controller.frame = self.frame
    output, self.previous = self.controller.update(self.cc.as_reader(), structs.CarControlSP(), self.cs, now)
    self.frame += 1
    return output, list(self.previous)

  def emission(self):
    for _ in range(2):
      output, messages = self.step()
      if any(a == 0x2B6 for a, _, _ in messages):
        return output, {a: self.decode(a, d) for a, d, _ in messages if a in (0x2B6, 0x2F6)}
    return output, {}

  def decode(self, address, data):
    return {name: round(get_raw_value(data, sig) * sig.factor + sig.offset, 6)
            for name, sig in self.packer.dbc.addr_to_msg[address].sigs.items()}


class TestLongitudinalReference(unittest.TestCase):
  def test_recorded_active_and_inactive_frames_repack_exactly(self):
    reference = json.loads((Path(__file__).parent / 'fixtures/longitudinal_reference.json').read_text())
    packer = CANPacker('psa_aee2010_r3')
    for sample in reference['samples']:
      with self.subTest(sample=sample['case']):
        name = packer.dbc.addr_to_msg[sample['address']].name
        builder = getattr(psacan, 'create_' + name)
        kwargs = {k: sample['signals'][k.upper()] for k in inspect.signature(builder).parameters if k not in ('packer', 'bus')}
        message = builder(packer, bus=1, **kwargs)
        self.assertEqual(message, (sample['address'], bytes.fromhex(sample['payload_hex']), 1))
        data = message[1]
        # Independent raw-byte checks: these are the counter/checksum positions observed in the logs.
        index, nibble_sum = (7, 12) if message[0] == 0x2B6 else (6, 8)
        self.assertEqual(data[index] >> 4, sample['signals']['COUNTER'])
        self.assertEqual(sum((b >> 4) + (b & 15) for b in data) % 16, nibble_sum)
        if message[0] == 0x2B6:
          self.assertEqual((data[6] >> 6) & 1, (data[7] >> 4) & 1)


class TestLongitudinalCommands(unittest.TestCase):
  def setUp(self):
    self.h = LongitudinalHarness()
    self.h.activate()

  def assert_inactive(self, values):
    self.assertIn(0x2B6, values)
    self.assertIn(0x2F6, values)
    b6, f6 = values[0x2B6], values[0x2F6]
    for field in ('POTENTIAL_WHEEL_TORQUE_REQUEST', 'WHEEL_TORQUE_REQUEST', 'MDD_DECEL_TYPE', 'MDD_DECEL_CONTROL_REQ', 'PREFILL_REQUEST'):
      self.assertEqual(b6[field], 0, field)
    self.assertEqual(f6['MDD_DECEL_CONTROL_REQ'], 0)
    self.assertEqual(b6['GMP_WHEEL_TORQUE'], -4000)
    self.assertEqual(b6['GMP_POTENTIAL_WHEEL_TORQUE'], -4000)
    self.assertEqual(b6['MDD_DESIRED_DECELERATION'], 2.05)
    self.assertEqual(b6['MIN_TIME_FOR_DESIRED_GEAR'], 0)

  def test_gmp_request_and_experimental_torque_encoding(self):
    output, values = self.h.emission()
    self.assertEqual(values[0x2B6]['WHEEL_TORQUE_REQUEST'], 1)
    self.assertEqual(values[0x2B6]['POTENTIAL_WHEEL_TORQUE_REQUEST'], 1)
    self.assertEqual(values[0x2B6]['GMP_WHEEL_TORQUE'], 424)
    self.assertEqual(values[0x2B6]['GMP_POTENTIAL_WHEEL_TORQUE'], 392)
    self.assertEqual(values[0x2B6]['MIN_TIME_FOR_DESIRED_GEAR'], 6.2)
    self.assertEqual(values[0x2B6]['ACC_STATUS'], 4)
    self.assertEqual(values[0x2F6]['MDD_DECEL_CONTROL_REQ'], 0)
    self.assertAlmostEqual(output.accel, 0.5)

  # [acc hold] - START
  def test_accelerator_hold_releases_actuation_and_resumes_current_target(self):
    for initial_accel in (0.5, -0.75):
      with self.subTest(initial_accel=initial_accel):
        self.h.cc.actuators.accel = initial_accel
        self.h.emission()
        self.h.cs.out.gasPressed = True
        # Sunnypilot keeps enabled during pedal override when DisengageOnAccelerator is off.
        self.h.cc.longActive = False
        for _ in range(20):
          output, values = self.h.emission()
          self.assertEqual(values[0x2B6]['ACC_STATUS'], 5)
          self.assertTrue(self.h.controller.acc_on_hold)
          self.assert_inactive(values)
          self.assertEqual(output.accel, 0)
        self.h.cs.out.gasPressed = False
        self.h.cc.longActive = True
        self.h.cc.actuators.accel = 0.125
        _, values = self.h.emission()
        self.assertFalse(self.h.controller.acc_on_hold)
        self.assertEqual(values[0x2B6]['ACC_STATUS'], 4)
        self.assertEqual(values[0x2B6]['GMP_WHEEL_TORQUE'], 240)

  def test_disengage_on_accelerator_does_not_resume_on_release(self):
    self.h.emission()
    # With DisengageOnAccelerator on, Sunnypilot also clears CC.enabled.
    self.h.cc.enabled = False
    self.h.cc.longActive = False
    for gas in (True, False):
      self.h.cs.out.gasPressed = gas
      _, values = self.h.emission()
      self.assertFalse(self.h.controller.acc_on_hold)
      self.assertNotIn(values[0x2B6]['ACC_STATUS'], (4, 5))
      self.assert_inactive(values)

  def test_brake_cancel_and_invalid_can_clear_hold(self):
    for reason in ('brake', 'cancel', 'cruise_off', 'invalid_can'):
      with self.subTest(reason=reason):
        self.h = LongitudinalHarness()
        self.h.activate()
        self.h.emission()
        self.h.cs.out.gasPressed = True
        self.h.cc.longActive = False
        self.h.emission()
        self.assertTrue(self.h.controller.acc_on_hold)
        if reason == 'brake':
          self.h.cs.out.brakePressed = True
        elif reason == 'cancel':
          self.h.cc.enabled = False
        elif reason == 'cruise_off':
          self.h.cs.out.cruiseState.enabled = False
        else:
          self.h.cs.out.canValid = False
        _, values = self.h.emission()
        self.assertFalse(self.h.controller.acc_on_hold)
        self.assert_inactive(values)
        self.h.cs.out.gasPressed = False
        _, values = self.h.emission()
        self.assertNotIn(values[0x2B6]['ACC_STATUS'], (4, 5))
        self.assert_inactive(values)

  def test_engagement_with_gas_already_pressed_creates_hold(self):
    self.h.cc.longActive = False
    self.h.cc.enabled = False
    self.h.cs.out.cruiseState.enabled = False
    self.h.cs.out.gasPressed = True
    _, values = self.h.emission()
    self.assertFalse(self.h.controller.acc_on_hold)
    self.assertNotIn(values[0x2B6]['ACC_STATUS'], (4, 5))
    self.assert_inactive(values)
    self.h.cc.enabled = True
    self.h.cs.out.cruiseState.enabled = True
    for _ in range(20):
      _, values = self.h.emission()
      self.assertTrue(self.h.controller.acc_on_hold)
      self.assertEqual(values[0x2B6]['ACC_STATUS'], 5)
      self.assert_inactive(values)
    self.h.cs.out.gasPressed = False
    self.h.cc.longActive = True
    _, values = self.h.emission()
    self.assertFalse(self.h.controller.acc_on_hold)
    self.assertEqual(values[0x2B6]['ACC_STATUS'], 4)

  def test_release_waits_for_longitudinal_authorization(self):
    self.h.emission()
    self.h.cs.out.gasPressed = True
    self.h.cc.longActive = False
    self.h.emission()
    self.h.cs.out.gasPressed = False
    _, values = self.h.emission()
    self.assertFalse(self.h.controller.acc_on_hold)
    self.assertNotIn(values[0x2B6]['ACC_STATUS'], (4, 5))
    self.assert_inactive(values)

  def test_cruise_off_blocks_stale_active_command(self):
    self.h.emission()
    self.h.cs.out.cruiseState.enabled = False
    _, values = self.h.emission()
    self.assert_inactive(values)

  def test_session_loss_clears_hold_and_stops_transmission(self):
    self.h.emission()
    self.h.cs.out.gasPressed = True
    self.h.cc.longActive = False
    self.h.emission()
    self.assertTrue(self.h.controller.acc_on_hold)
    now = self.h.frame * 10_000_000
    self.h.controller.process_radar_can([(now, [(0x2F6, bytes.fromhex('00ff8600f8600f00'), 1)])])
    output, messages = self.h.step()
    self.assertFalse(self.h.controller.acc_on_hold)
    self.assertEqual(output.accel, 0)
    self.assertFalse(any(a in RADAR_IDS for a, _, _ in messages))
  # [acc hold] - END

  def test_acc_waiting_before_engagement_and_after_brake_release(self):
    # Stock routes 3a/49 announce Waiting before the BSI activation request.
    self.h.cs.out.standstill = False
    self.h.cs.out.vEgo = 15.0
    self.h.cs.out.vEgoRaw = 15.0
    for enabled, brake_pressed, status in ((False, False, 3), (True, False, 4),
                                           (False, True, 2), (False, False, 3)):
      with self.subTest(enabled=enabled, brake_pressed=brake_pressed):
        self.h.cc.enabled = enabled
        self.h.cc.longActive = enabled
        self.h.cs.out.cruiseState.enabled = enabled
        self.h.cs.out.brakePressed = brake_pressed
        _, values = self.h.emission()
        self.assertEqual(values[0x2B6]['ACC_STATUS'], status)
        if not enabled:
          self.assert_inactive(values)
        else:
          self.assertEqual(values[0x2B6]['WHEEL_TORQUE_REQUEST'], 1)

  def test_acc_waiting_speed_parameter_uses_raw_can_speed(self):
    self.assertAlmostEqual(self.h.controller.CP.minEnableSpeed, 27 * CV.KPH_TO_MS)
    self.h.cc.enabled = False
    self.h.cc.longActive = False
    self.h.cs.out.cruiseState.enabled = False
    self.h.cs.out.standstill = False
    for threshold_kph in (27, 20):
      self.h.controller.CP.minEnableSpeed = threshold_kph * CV.KPH_TO_MS
      for speed_kph, brake, expected in ((threshold_kph - 0.1, False, 2),
                                       (threshold_kph, False, 3),
                                       (threshold_kph + 0.1, False, 3),
                                       (threshold_kph + 0.1, True, 2),
                                       (threshold_kph - 0.1, False, 2)):
        with self.subTest(threshold_kph=threshold_kph, speed_kph=speed_kph, brake=brake):
          self.h.cs.out.vEgoRaw = speed_kph * CV.KPH_TO_MS
          # Deliberately disagree with the raw speed on either side of the threshold.
          self.h.cs.out.vEgo = (threshold_kph + (5 if expected == 2 else -5)) * CV.KPH_TO_MS
          self.h.cs.out.brakePressed = brake
          _, values = self.h.emission()
          self.assertEqual(values[0x2B6]['ACC_STATUS'], expected)
          self.assert_inactive(values)

  def test_active_acc_stays_active_below_waiting_speed(self):
    self.h.cs.out.vEgoRaw = 20 * CV.KPH_TO_MS
    self.h.cs.out.vEgo = self.h.cs.out.vEgoRaw
    _, values = self.h.emission()
    self.assertEqual(values[0x2B6]['ACC_STATUS'], 4)
    self.assertEqual(values[0x2B6]['WHEEL_TORQUE_REQUEST'], 1)

  def test_display_set_speed_does_not_change_longitudinal_can_commands(self):
    for accel in (0.5, -0.75):
      commands = []
      for display_kph in (47, 50):
        h = LongitudinalHarness()
        h.cc.actuators.accel = accel
        h.cs.out.cruiseState.speed = 47 * CV.KPH_TO_MS
        h.cs.out.cruiseState.speedCluster = display_kph * CV.KPH_TO_MS
        h.cc.hudControl.setSpeed = h.cs.out.cruiseState.speedCluster
        h.activate()
        output, values = h.emission()
        commands.append((output.accel, values))
      self.assertEqual(commands[0], commands[1])

  # [torque calibration] - START
  def test_grade_adjusts_both_torque_fields_without_changing_acceleration_target(self):
    # Same 0.5 m/s2 target, +/-0.25 m/s2 gravity contribution: independent
    # reference points 0.25/0.50/0.75, with potential quantized to 4 Nm by DBC.
    for pitch, wheel, potential in ((0.0, 424, 392),
                                    (math.asin(0.25 / 9.81), 547, 500),
                                    (-math.asin(0.25 / 9.81), 301, 280)):
      with self.subTest(pitch=pitch):
        self.h.cc.orientationNED = [0.0, pitch, 0.0]
        output, values = self.h.emission()
        self.assertEqual(values[0x2B6]['GMP_WHEEL_TORQUE'], wheel)
        self.assertEqual(values[0x2B6]['GMP_POTENTIAL_WHEEL_TORQUE'], potential)
        self.assertEqual(values[0x2B6]['MDD_DECEL_CONTROL_REQ'], 0)
        self.assertAlmostEqual(output.accel, 0.5)

  def test_interpolation_uses_separate_calibrated_fields(self):
    self.h.cc.actuators.accel = 0.125
    _, values = self.h.emission()
    self.assertEqual(values[0x2B6]['GMP_WHEEL_TORQUE'], 240)
    self.assertEqual(values[0x2B6]['GMP_POTENTIAL_WHEEL_TORQUE'], 224)

  def test_grade_compensation_stays_within_existing_torque_bounds(self):
    for pitch, wheel, potential in ((math.pi / 2, 1000, 1000), (-math.pi / 2, -400, -400)):
      with self.subTest(pitch=pitch):
        self.h.cc.orientationNED = [0.0, pitch, 0.0]
        output, values = self.h.emission()
        self.assertEqual(values[0x2B6]['GMP_WHEEL_TORQUE'], wheel)
        self.assertEqual(values[0x2B6]['GMP_POTENTIAL_WHEEL_TORQUE'], potential)
        self.assertAlmostEqual(output.accel, 0.5)

  def test_invalid_pitch_clears_previous_gmp_request(self):
    for pitch in (math.nan, math.inf, -math.inf):
      with self.subTest(pitch=pitch):
        h = LongitudinalHarness()
        h.activate()
        h.emission()
        h.cc.orientationNED = [0.0, pitch, 0.0]
        output, values = h.emission()
        self.assert_inactive(values)
        self.assertEqual(output.accel, 0)

  def test_grade_does_not_modify_direct_braking_request(self):
    self.h.cc.actuators.accel = -0.75
    for pitch in (-0.1, 0.1, math.nan):
      with self.subTest(pitch=pitch):
        self.h.cc.orientationNED = [0.0, pitch, 0.0]
        output, values = self.h.emission()
        self.assertEqual(values[0x2B6]['MDD_DESIRED_DECELERATION'], -0.75)
        self.assertEqual(values[0x2B6]['GMP_WHEEL_TORQUE'], -4000)
        self.assertEqual(values[0x2B6]['GMP_POTENTIAL_WHEEL_TORQUE'], -4000)
        self.assertEqual(values[0x2F6]['MDD_DECEL_CONTROL_REQ'], 1)
        self.assertAlmostEqual(output.accel, -0.75)
  # [torque calibration] - END

  def test_braking_flags_agree_and_gmp_request_is_removed(self):
    self.h.emission()
    self.h.cc.actuators.accel = -0.75
    output, values = self.h.emission()
    self.assertEqual(values[0x2B6]['MDD_DESIRED_DECELERATION'], -0.75)
    self.assertEqual(values[0x2B6]['POTENTIAL_WHEEL_TORQUE_REQUEST'], 2)
    self.assertEqual(values[0x2B6]['WHEEL_TORQUE_REQUEST'], 0)
    self.assertEqual(values[0x2B6]['GMP_WHEEL_TORQUE'], -4000)
    self.assertEqual(values[0x2B6]['GMP_POTENTIAL_WHEEL_TORQUE'], -4000)
    self.assertEqual(values[0x2B6]['MDD_DECEL_CONTROL_REQ'], 1)
    self.assertEqual(values[0x2B6]['MDD_DECEL_TYPE'], 1)
    self.assertEqual(values[0x2B6]['MIN_TIME_FOR_DESIRED_GEAR'], 0)
    self.assertEqual(values[0x2F6]['MDD_DECEL_CONTROL_REQ'], 1)
    self.assertAlmostEqual(output.accel, -0.75)

  # [light braking] - START
  def test_braking_remains_continuous_across_old_threshold_and_to_zero(self):
    for accel in (-0.75, -0.49, -0.51, -0.15, -0.05, 0):
      with self.subTest(accel=accel):
        self.h.cc.actuators.accel = accel
        output, values = self.h.emission()
        b6, f6 = values[0x2B6], values[0x2F6]
        self.assertEqual(b6['MDD_DECEL_CONTROL_REQ'], 1)
        self.assertEqual(f6['MDD_DECEL_CONTROL_REQ'], 1)
        self.assertAlmostEqual(b6['MDD_DESIRED_DECELERATION'], round(accel / 0.05) * 0.05)
        self.assertEqual(b6['WHEEL_TORQUE_REQUEST'], 0)
        self.assertEqual(b6['GMP_WHEEL_TORQUE'], -4000)
        self.assertAlmostEqual(output.accel, accel)

  def test_downhill_enters_light_braking_without_previous_strong_request(self):
    for accel in (-0.4, -0.15, 0):
      with self.subTest(accel=accel):
        h = LongitudinalHarness()
        h.activate()
        h.cc.orientationNED = [0, -0.085, 0]  # Median pitch in route 56's repeated-braking interval.
        h.cc.actuators.accel = accel
        _, values = h.emission()
        self.assertEqual(values[0x2B6]['MDD_DECEL_CONTROL_REQ'], 1)
        self.assertAlmostEqual(values[0x2B6]['MDD_DESIRED_DECELERATION'], accel)

  def test_positive_target_releases_brake_immediately(self):
    self.h.cc.actuators.accel = -0.75
    self.h.emission()
    for accel in (0.01, 0):
      self.h.cc.actuators.accel = accel
      _, values = self.h.emission()
      self.assertEqual(values[0x2B6]['MDD_DECEL_CONTROL_REQ'], 0)
      self.assertEqual(values[0x2B6]['WHEEL_TORQUE_REQUEST'], 1)

  def test_pedals_disengagement_and_invalid_inputs_reset_braking_mode(self):
    for reason in ('gas', 'brake', 'enabled', 'longActive', 'cruise', 'canValid', 'accel'):
      with self.subTest(reason=reason):
        self.h = LongitudinalHarness()
        self.h.activate()
        self.h.cc.actuators.accel = -0.75
        self.h.emission()
        obj, field, value = {
          'gas': (self.h.cs.out, 'gasPressed', True),
          'brake': (self.h.cs.out, 'brakePressed', True),
          'enabled': (self.h.cc, 'enabled', False),
          'longActive': (self.h.cc, 'longActive', False),
          'cruise': (self.h.cs.out.cruiseState, 'enabled', False),
          'canValid': (self.h.cs.out, 'canValid', False),
          'accel': (self.h.cc.actuators, 'accel', math.nan),
        }[reason]
        previous = getattr(obj, field)
        setattr(obj, field, value)
        _, values = self.h.emission()
        self.assert_inactive(values)
        setattr(obj, field, previous)
        self.h.cc.actuators.accel = -0.15  # Level road: no stale braking mode after an interruption.
        _, values = self.h.emission()
        self.assertEqual(values[0x2B6]['MDD_DECEL_CONTROL_REQ'], 0)
  # [light braking] - END

  # [brake limit] - START
  def test_disengagement_and_pedals_remove_previous_requests(self):
    for field in ('longActive', 'enabled', 'gasPressed', 'brakePressed'):
      for accel in (0.5, -0.75, -2):
        with self.subTest(field=field, accel=accel):
          h = LongitudinalHarness()
          h.activate()
          h.cc.actuators.accel = accel
          h.emission()
          target = h.cc if field in ('longActive', 'enabled') else h.cs.out
          setattr(target, field, field in ('gasPressed', 'brakePressed'))
          output, values = h.emission()
          self.assert_inactive(values)
          self.assertEqual(output.accel, 0)
  # [brake limit] - END

  def test_non_finite_acceleration_cannot_leave_a_request_latched(self):
    for accel in (math.nan, math.inf, -math.inf):
      with self.subTest(accel=accel):
        h = LongitudinalHarness()
        h.activate()
        h.emission()
        h.cc.actuators.accel = accel
        output, values = h.emission()
        self.assert_inactive(values)
        self.assertEqual(output.accel, 0)

  # [brake limit] - START
  def test_clamp_and_braking_boundary(self):
    for accel, expected_accel, wheel, braking in ((10, 2, 1000, 0), (-10, -2, -4000, 1), (-0.5, -0.5, -300, 0)):
      with self.subTest(accel=accel):
        # [light braking] - START
        # Entry boundary: do not inherit a braking episode from the previous case.
        self.h = LongitudinalHarness()
        self.h.activate()
        # [light braking] - END
        self.h.cc.actuators.accel = accel
        output, values = self.h.emission()
        self.assertAlmostEqual(output.accel, expected_accel)
        self.assertEqual(values[0x2B6]['GMP_WHEEL_TORQUE'], wheel)
        self.assertEqual(values[0x2B6]['MDD_DECEL_CONTROL_REQ'], braking)
        self.assertEqual(values[0x2F6]['MDD_DECEL_CONTROL_REQ'], braking)

  def test_direct_braking_has_its_own_limit_independent_of_torque_and_grade(self):
    for requested, applied, encoded in ((-1.84, -1.84, -1.85), (-2, -2, -2), (-2.05, -2, -2), (-10, -2, -2)):
      for pitch in (-0.1, 0, 0.1):
        with self.subTest(requested=requested, pitch=pitch):
          self.h.cc.actuators.accel = requested
          self.h.cc.orientationNED = [0, pitch, 0]
          output, values = self.h.emission()
          b6, f6 = values[0x2B6], values[0x2F6]
          self.assertAlmostEqual(output.accel, applied)
          self.assertAlmostEqual(b6['MDD_DESIRED_DECELERATION'], encoded)
          self.assertEqual(b6['MDD_DECEL_CONTROL_REQ'], 1)
          self.assertEqual(f6['MDD_DECEL_CONTROL_REQ'], 1)
          self.assertEqual(b6['MDD_DECEL_TYPE'], 1)
          self.assertEqual(b6['POTENTIAL_WHEEL_TORQUE_REQUEST'], 2)
          self.assertEqual(b6['WHEEL_TORQUE_REQUEST'], 0)
          self.assertEqual(b6['GMP_WHEEL_TORQUE'], -4000)
          self.assertEqual(b6['GMP_POTENTIAL_WHEEL_TORQUE'], -4000)
  # [brake limit] - END

  # [radar optin] - START
  def test_disabled_longitudinal_never_programs_or_substitutes_stock_radar(self):
    for kwargs in ({'dashcam': True}, {'passive': True}, {'experimental': False}, {'safety_flag': False}):
      with self.subTest(profile=kwargs):
        h = LongitudinalHarness(**kwargs)
        for tick in range(350):
          now = h.frame * 10_000_000
          h.controller.process_radar_can([(now, [(0x2B6, bytes.fromhex('fe0000020000030a'), 1)])])
          if tick == 10:
            h.controller.process_radar_can([(now, [(0x696, bytes.fromhex('06500200c80014'), 1)])])
          h.cs.out.standstill = tick < 200
          _, messages = h.step()
          self.assertFalse(any(a in RADAR_IDS or a == 0x6B6 for a, _, _ in messages), messages)
        self.assertFalse(h.controller.artiv_programming_requested)
        self.assertFalse(h.controller.radar_active)
  # [radar optin] - END

  def test_alpha_long_selects_peugeot_profile_and_matching_safety(self):
    for platform in CAR:
      for alpha_long in (False, True):
        with self.subTest(platform=platform, alpha_long=alpha_long):
          cp = CarInterface.get_params(platform, {0: {}, 1: {}, 2: {}}, [], alpha_long, False, False)
          available = platform == CAR.PSA_PEUGEOT_3008
          enabled = available and alpha_long
          self.assertEqual(cp.alphaLongitudinalAvailable, available)
          self.assertEqual(cp.dashcamOnly, not enabled)
          self.assertEqual(cp.openpilotLongitudinalControl, enabled)
          self.assertEqual(bool(cp.safetyConfigs[0].safetyParam & PSA_LONG_CONTROL), enabled)
          self.assertTrue(cp.pcmCruise)
          cp_sp = CarInterface.get_params_sp(cp, platform, {0: {}, 1: {}, 2: {}}, [], alpha_long, False, False)
          self.assertTrue(cp_sp.pcmCruiseSpeed)
          controller = CarController({Bus.main: 'psa_aee2010_r3'}, cp, cp_sp)
          self.assertEqual(controller.longitudinal_enabled, enabled)

  def test_alpha_long_params_reach_actual_command_generation(self):
    cp = CarInterface.get_params(CAR.PSA_PEUGEOT_3008, {0: {}, 1: {}, 2: {}}, [], True, False, False)
    cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.PSA_PEUGEOT_3008)
    self.h = LongitudinalHarness()
    self.h.controller = CarController({Bus.main: 'psa_aee2010_r3'}, cp, cp_sp)
    self.h.activate()
    _, values = self.h.emission()
    self.assertEqual(values[0x2B6]['WHEEL_TORQUE_REQUEST'], 1)
    self.h.cc.actuators.accel = -0.75
    _, values = self.h.emission()
    self.assertEqual(values[0x2B6]['MDD_DECEL_CONTROL_REQ'], 1)
    self.assertEqual(values[0x2F6]['MDD_DECEL_CONTROL_REQ'], 1)

  def test_takeover_during_braking_preserves_both_brake_requests(self):
    self.h.cc.actuators.accel = -0.75
    self.h.controller.takeover_req = 1
    for takeover in (1, 1, 0):
      _, values = self.h.emission()
      self.assertEqual(values[0x2F6]['REQUEST_TAKEOVER'], takeover)
      self.assertEqual(values[0x2B6]['MDD_DECEL_CONTROL_REQ'], 1)
      self.assertEqual(values[0x2F6]['MDD_DECEL_CONTROL_REQ'], 1)

  def test_no_radar_session_no_substitute_frames_and_no_reported_actuation(self):
    h = LongitudinalHarness()
    output, messages = h.step()
    self.assertFalse(any(a in RADAR_IDS for a, _, _ in messages))
    self.assertEqual(output.accel, 0)

  def test_takeover_shares_one_frame_and_counter_without_enabling_longitudinal(self):
    self.h.cc.longActive = False
    self.h.controller.takeover_req = 2
    counts = Counter()
    counters, takeover = [], []
    for _ in range(100):
      _, messages = self.h.step()
      self.assertLessEqual(sum(a == 0x2F6 for a, _, _ in messages), 1)
      for a, data, _ in messages:
        counts[a] += 1
        if a == 0x2F6:
          values = self.h.decode(a, data)
          counters.append(values['COUNTER'])
          takeover.append(values['REQUEST_TAKEOVER'])
          self.assertEqual(values['MDD_DECEL_CONTROL_REQ'], 0)
    self.assertEqual(counts[0x2B6], 50)
    self.assertEqual(counts[0x2F6], 50)
    self.assertEqual(counts[0x4F6], 10)
    self.assertEqual(counts[0x796], 1)
    self.assertEqual(counters, [i % 16 for i in range(50)])
    self.assertEqual(takeover, [2, 2] + [0] * 48)


if __name__ == '__main__':
  unittest.main()
# [psa longitudinal] - END
