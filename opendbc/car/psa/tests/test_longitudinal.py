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
from opendbc.car.psa import psacan
from opendbc.car.psa.carcontroller import CarController
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.values import CAR, PSA_LONG_CONTROL

RADAR_IDS = {0x2B6, 0x2F6, 0x4F6, 0x796}


class LongitudinalHarness:
  def __init__(self, *, dashcam=False, passive=False, experimental=True, safety_flag=True):
    cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_3008)
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
    radar = self.controller.neutral_radar
    radar.process_can([(10_000_000_000, [(0x2B6, bytes.fromhex('fe0000020000030a'), 1)])])
    self.step()
    radar.process_can([(10_067_000_000, [(0x696, bytes.fromhex('06500200c80014'), 1)])])
    self.frame = 1011

  def step(self):
    now = self.frame * 10_000_000
    rx = [(0x212, bytes(8), 1)] + [(a, d, 129) for a, d, _ in self.previous if a in RADAR_IDS]
    if any(a == 0x6B6 and d == b'\x02\x3e\x00' for a, d, _ in self.previous):
      rx.append((0x696, b'\x02\x7e\x00', 1))
    self.controller.neutral_radar.process_can([(now, rx)])
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
    self.assertEqual(values[0x2B6]['GMP_WHEEL_TORQUE'], 350)
    self.assertEqual(values[0x2B6]['GMP_POTENTIAL_WHEEL_TORQUE'], 352)
    self.assertEqual(values[0x2B6]['MIN_TIME_FOR_DESIRED_GEAR'], 6.2)
    self.assertEqual(values[0x2B6]['ACC_STATUS'], 4)
    self.assertEqual(values[0x2F6]['MDD_DECEL_CONTROL_REQ'], 0)
    self.assertAlmostEqual(output.accel, 0.5)

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

  def test_disengagement_and_pedals_remove_previous_requests(self):
    for field in ('longActive', 'enabled', 'gasPressed', 'brakePressed'):
      for accel in (0.5, -0.75):
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

  def test_clamp_and_braking_boundary(self):
    for accel, expected_accel, wheel, braking in ((10, 2, 1000, 0), (-10, -1, -4000, 1), (-0.5, -0.5, -300, 0)):
      with self.subTest(accel=accel):
        self.h.cc.actuators.accel = accel
        output, values = self.h.emission()
        self.assertAlmostEqual(output.accel, expected_accel)
        self.assertEqual(values[0x2B6]['GMP_WHEEL_TORQUE'], wheel)
        self.assertEqual(values[0x2B6]['MDD_DECEL_CONTROL_REQ'], braking)
        self.assertEqual(values[0x2F6]['MDD_DECEL_CONTROL_REQ'], braking)

  def test_dashcam_passive_and_default_profiles_never_actuate(self):
    for kwargs in ({'dashcam': True}, {'passive': True}, {'experimental': False}, {'safety_flag': False}):
      with self.subTest(profile=kwargs):
        h = LongitudinalHarness(**kwargs)
        h.activate()
        _, values = h.emission()
        self.assert_inactive(values)

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
