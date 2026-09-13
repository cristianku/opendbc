# [psa longitudinal] - START
# [light braking] - START
import json
from pathlib import Path
# [light braking] - END
import unittest

from opendbc.can.packer import CANPacker
from opendbc.car import structs
from opendbc.car.psa.tests.test_longitudinal import LongitudinalHarness, RADAR_IDS
from opendbc.car.psa.values import PSA_LONG_CONTROL
from opendbc.safety.tests.libsafety import libsafety_py


class TestPsaLongitudinalSafety(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.packer = CANPacker('psa_aee2010_r3')
    self.configure()
    self.neutral = dict(MDD_DESIRED_DECELERATION=2.05, POTENTIAL_WHEEL_TORQUE_REQUEST=0,
                        MIN_TIME_FOR_DESIRED_GEAR=0, GMP_POTENTIAL_WHEEL_TORQUE=-4000,
                        ACC_STATUS=2, GMP_WHEEL_TORQUE=-4000, WHEEL_TORQUE_REQUEST=0,
                        AUTO_BRAKING_STATUS=3, MDD_DECEL_TYPE=0, MDD_DECEL_CONTROL_REQ=0,
                        GEAR_TYPE=0, PREFILL_REQUEST=0, COUNTER=0)
    self.gmp = dict(self.neutral, POTENTIAL_WHEEL_TORQUE_REQUEST=1, WHEEL_TORQUE_REQUEST=1,
                    GMP_POTENTIAL_WHEEL_TORQUE=352, GMP_WHEEL_TORQUE=350,
                    MIN_TIME_FOR_DESIRED_GEAR=6.2, ACC_STATUS=4)
    self.braking = dict(self.neutral, POTENTIAL_WHEEL_TORQUE_REQUEST=2, MDD_DECEL_TYPE=1,
                        MDD_DECEL_CONTROL_REQ=1, MDD_DESIRED_DECELERATION=-0.75, ACC_STATUS=4)
    self.display = dict(AUTO_BRAKING_STATUS=3, ARC_STATUS=6, INTER_VEHICLE_DISTANCE=255.5,
                        DISPLAY_INTERVEHICLE_TIME=6.2)

  def configure(self, flag=PSA_LONG_CONTROL, controls=True):
    self.assertEqual(self.safety.set_safety_hooks(structs.CarParams.SafetyModel.psa, flag), 0)
    self.safety.init_tests()
    self.safety.set_controls_allowed(controls)

  def message(self, address, values, bus=1):
    return self.packer.make_can_msg(self.packer.dbc.addr_to_msg[address].name, bus, values)

  def tx(self, message):
    address, data, bus = message
    return self.safety.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, data))

  def test_active_requests_require_flag_and_controls(self):
    for flag in (0, PSA_LONG_CONTROL):
      for controls in (False, True):
        self.configure(flag, controls)
        for address, values in ((0x2B6, self.gmp), (0x2B6, self.braking),
                                (0x2F6, dict(self.display, MDD_DECEL_CONTROL_REQ=1))):
          with self.subTest(flag=flag, controls=controls, address=address, values=values):
            self.assertEqual(self.tx(self.message(address, values)), bool(flag and controls))
        self.assertTrue(self.tx(self.message(0x2B6, self.neutral)))
        self.assertTrue(self.tx(self.message(0x2F6, self.display)))

  def test_pedals_inhibit_even_if_controls_remain_allowed(self):
    for pedal in ('gas', 'brake'):
      with self.subTest(pedal=pedal):
        self.configure()
        if pedal == 'gas':
          # [acc hold] - START
          packet = libsafety_py.make_CANPacket(0x228, 0, b'\x00\x00\x01' + bytes(5))
          # [acc hold] - END
        else:
          packet = libsafety_py.make_CANPacket(0x412, 2, b'\x20' + bytes(7))
        self.assertTrue(self.safety.safety_rx_hook(packet))
        self.assertTrue(self.safety.get_gas_pressed_prev() if pedal == 'gas' else self.safety.get_brake_pressed_prev())
        self.safety.set_controls_allowed(True)
        self.assertFalse(self.tx(self.message(0x2B6, self.gmp)))
        self.assertFalse(self.tx(self.message(0x2B6, self.braking)))
        self.assertFalse(self.tx(self.message(0x2F6, dict(self.display, MDD_DECEL_CONTROL_REQ=1))))
        self.assertTrue(self.tx(self.message(0x2B6, self.neutral)))

  # [acc hold] - START
  def test_hold_allows_only_neutral_payload_in_longitudinal_profile(self):
    for flag in (0, PSA_LONG_CONTROL):
      for controls in (False, True):
        for gas in (False, True):
          with self.subTest(flag=flag, controls=controls, gas=gas):
            self.configure(flag, controls)
            self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x228, 0, bytes([0, 0, int(gas), 0, 0, 0, 0, 0])))
            hold = dict(self.neutral, ACC_STATUS=5)
            self.assertEqual(self.tx(self.message(0x2B6, hold)), bool(flag))
            for field, value in (('GMP_WHEEL_TORQUE', 0), ('GMP_POTENTIAL_WHEEL_TORQUE', 0),
                                 ('WHEEL_TORQUE_REQUEST', 1), ('POTENTIAL_WHEEL_TORQUE_REQUEST', 1),
                                 ('MIN_TIME_FOR_DESIRED_GEAR', 6.2), ('MDD_DESIRED_DECELERATION', -0.75),
                                 ('MDD_DECEL_CONTROL_REQ', 1), ('MDD_DECEL_TYPE', 1), ('PREFILL_REQUEST', 1)):
              self.assertFalse(self.tx(self.message(0x2B6, dict(hold, **{field: value}))), field)
            for active in (self.gmp, self.braking):
              self.assertFalse(self.tx(self.message(0x2B6, dict(active, ACC_STATUS=5))))

  def test_physical_pedal_source_is_selected_and_reset_with_profile(self):
    for flag in (PSA_LONG_CONTROL, 0, PSA_LONG_CONTROL):
      with self.subTest(flag=flag):
        self.configure(flag)
        address, bus, length, byte = (0x228, 0, 8, 2) if flag else (0x56E, 2, 5, 3)
        data = bytearray(length)
        data[byte] = 1
        self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, data)))
        self.assertTrue(self.safety.get_gas_pressed_prev())
        # The unused source cannot overwrite the real pedal with a constant zero.
        other_address, other_bus, other_length = (0x56E, 2, 5) if flag else (0x228, 0, 8)
        self.safety.safety_rx_hook(libsafety_py.make_CANPacket(other_address, other_bus, bytes(other_length)))
        self.assertTrue(self.safety.get_gas_pressed_prev())
        self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, bytes(length))))
        self.assertFalse(self.safety.get_gas_pressed_prev())

  def test_controller_hold_frames_pass_safety_with_physical_pedal(self):
    for disengage in (False, True):
      with self.subTest(disengage=disengage):
        self.configure()
        h = LongitudinalHarness()
        h.activate()
        counters = []
        for index in range(60):
          gas = 10 <= index < 40
          h.cs.out.gasPressed = gas
          h.cc.enabled = not (disengage and index >= 10)
          h.cc.longActive = h.cc.enabled and not gas
          self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x228, 0, bytes([0, 0, int(gas), 0, 0, 0, 0, 0])))
          if disengage and index == 10:
            self.safety.set_controls_allowed(False)
          self.assertEqual(self.safety.get_controls_allowed(), h.cc.enabled)
          _, messages = h.step()
          for message in messages:
            if message[0] in RADAR_IDS:
              self.assertTrue(self.tx(message), (index, message))
            if message[0] == 0x2B6:
              values = h.decode(message[0], message[1])
              expected = 2 if disengage and index >= 10 else (5 if gas else 4)
              self.assertEqual(values['ACC_STATUS'], expected)
              counters.append(values['COUNTER'])
        self.assertEqual(counters, [i % 16 for i in range(len(counters))])

  def test_missing_or_stale_physical_pedal_invalidates_rx_checks(self):
    for now in (2_000_000, 3_000_001):
      self.safety.set_timer(now)
      for address, bus in ((0x452, 1), (0x30D, 0), (0x38D, 0), (0x2F5, 0), (0x412, 2)):
        address, data, bus = self.message(address, {}, bus)
        self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, data)))
      # Legacy DRIVER traffic must neither replace nor refresh the physical pedal check.
      self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x56E, 2, bytes(5)))
      self.safety.set_controls_allowed(True)
      self.safety.safety_tick_current_safety_config()
      self.assertFalse(self.safety.safety_config_valid())
      self.assertFalse(self.safety.get_controls_allowed())
      self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x228, 0, bytes(8))))
      self.safety.safety_tick_current_safety_config()
      self.assertTrue(self.safety.safety_config_valid())

  def test_wrong_bus_or_length_cannot_release_physical_pedal(self):
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x228, 0, b'\x00\x00\x01' + bytes(5)))
    for bus, length in ((1, 8), (2, 8), (0, 7)):
      self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x228, bus, bytes(length)))
      self.assertTrue(self.safety.get_gas_pressed_prev())
      self.assertFalse(self.tx(self.message(0x2B6, self.gmp)))
  # [acc hold] - END

  def test_torque_and_deceleration_boundaries(self):
    for field, quantum in (('GMP_WHEEL_TORQUE', 1), ('GMP_POTENTIAL_WHEEL_TORQUE', 4)):
      for torque in (-400-quantum, -400, 1000, 1000+quantum):
        with self.subTest(field=field, torque=torque):
          self.assertEqual(self.tx(self.message(0x2B6, dict(self.gmp, **{field: torque}))), -400 <= torque <= 1000)
    # [light braking] - START
    for accel in (-1.05, -1, -0.55, -0.5, -0.45, -0.15, -0.05, 0, 0.05, 2.05):
      with self.subTest(accel=accel):
        self.assertEqual(self.tx(self.message(0x2B6, dict(self.braking, MDD_DESIRED_DECELERATION=accel))), -1 <= accel <= 0)
    # [light braking] - END

  # [light braking] - START
  def test_light_braking_and_zero_keep_all_authorization_gates(self):
    for accel in (-0.45, -0.15, -0.05, 0):
      for flag in (0, PSA_LONG_CONTROL):
        for controls in (False, True):
          for pedal in ('none', 'gas', 'brake'):
            with self.subTest(accel=accel, flag=flag, controls=controls, pedal=pedal):
              self.configure(flag, controls)
              if pedal == 'gas':
                self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x228, 0, b'\x00\x00\x01' + bytes(5)))
              elif pedal == 'brake':
                self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x412, 2, b'\x20' + bytes(7)))
              values = dict(self.braking, MDD_DESIRED_DECELERATION=accel)
              self.assertEqual(self.tx(self.message(0x2B6, values)), bool(flag and controls and pedal == 'none'))
              for changes in ({'ACC_STATUS': 5}, {'WHEEL_TORQUE_REQUEST': 1}, {'PREFILL_REQUEST': 1}):
                self.assertFalse(self.tx(self.message(0x2B6, dict(values, **changes))))

  def test_recorded_downhill_targets_keep_braking_and_pass_safety(self):
    fixture_path = Path(__file__).parents[2] / 'car/psa/tests/fixtures/downhill_route56.json'
    reference = json.loads(fixture_path.read_text())
    h = LongitudinalHarness()
    h.activate()
    h.cs.out.vEgo = h.cs.out.vEgoRaw = 17.74
    h.cs.out.standstill = False
    counters = []
    for index, (route_seconds, accel, pitch) in enumerate(reference['samples']):
      h.cc.actuators.accel = accel
      h.cc.orientationNED = [0, pitch, 0]
      next_time = reference['samples'][index + 1][0] if index + 1 < len(reference['samples']) else route_seconds + 0.05
      for _ in range(max(1, round((next_time - route_seconds) / 0.01))):
        _, messages = h.step()
        for message in messages:
          if message[0] in RADAR_IDS:
            self.assertTrue(self.tx(message), (route_seconds, message))
          if message[0] == 0x2B6:
            values = h.decode(message[0], message[1])
            self.assertEqual(values['ACC_STATUS'], 4)
            self.assertEqual(values['MDD_DECEL_CONTROL_REQ'], 1, route_seconds)
            self.assertEqual(values['WHEEL_TORQUE_REQUEST'], 0)
            self.assertAlmostEqual(values['MDD_DESIRED_DECELERATION'], accel, delta=0.05)
            counters.append(values['COUNTER'])
          if message[0] == 0x2F6:
            self.assertEqual(h.decode(message[0], message[1])['MDD_DECEL_CONTROL_REQ'], 1)
    self.assertGreater(len(counters), 200)
    self.assertEqual(counters, [i % 16 for i in range(len(counters))])
  # [light braking] - END

  def test_incompatible_and_unimplemented_requests_rejected(self):
    cases = [(self.neutral, {'GMP_WHEEL_TORQUE': 0}),
             (self.neutral, {'GMP_POTENTIAL_WHEEL_TORQUE': 0}),
             (self.neutral, {'MDD_DESIRED_DECELERATION': 0}),
             (self.gmp, {'WHEEL_TORQUE_REQUEST': 0}),
             (self.gmp, {'POTENTIAL_WHEEL_TORQUE_REQUEST': 0}),
             (self.gmp, {'MDD_DECEL_CONTROL_REQ': 1}),
             (self.gmp, {'ACC_STATUS': 2}),
             (self.braking, {'WHEEL_TORQUE_REQUEST': 1}),
             (self.braking, {'MDD_DECEL_TYPE': 2}),
             (self.braking, {'MDD_DECEL_CONTROL_REQ': 0}),
             (self.braking, {'MIN_TIME_FOR_DESIRED_GEAR': 6.2})]
    for base, changes in cases:
      with self.subTest(changes=changes):
        self.assertFalse(self.tx(self.message(0x2B6, dict(base, **changes))))
    for values in (self.neutral, self.gmp, self.braking):
      for changes in ({'PREFILL_REQUEST': 1}, {'AUTO_BRAKING_STATUS': 5}, {'GEAR_TYPE': 1}):
        self.assertFalse(self.tx(self.message(0x2B6, dict(values, **changes))))
    for field in ('DRIVE_AWAY_REQUEST', 'AEB_ENABLED', 'AUTO_BRAKING_IN_PROGRESS'):
      self.assertFalse(self.tx(self.message(0x2F6, dict(self.display, **{field: 1}))))

  def test_takeover_is_independent_of_longitudinal_authorization(self):
    for controls in (False, True):
      self.configure(controls=controls)
      for takeover in range(4):
        self.assertEqual(self.tx(self.message(0x2F6, dict(self.display, REQUEST_TAKEOVER=takeover))), takeover < 3)

  def test_invalid_checksums_bus_and_length(self):
    for address, values, checksum_byte in ((0x2B6, self.neutral, 7), (0x2F6, self.display, 6)):
      message = self.message(address, values)
      data = bytearray(message[1])
      data[checksum_byte] ^= 1
      self.assertFalse(self.tx((address, data, 1)))
      self.assertFalse(self.tx((address, message[1], 0)))
      self.assertFalse(self.tx((address, message[1][:-1], 1)))

  def test_controller_frames_pass_and_keep_order_through_transitions(self):
    h = LongitudinalHarness()
    h.activate()
    counters = []
    for index in range(70):
      h.cc.actuators.accel = (0.5, -0.75, -0.501, 2, -1)[index // 14]
      h.cc.longActive = index < 64
      h.controller.takeover_req = 2 if index == 16 else h.controller.takeover_req
      _, messages = h.step()
      radar = [(a, d, b) for a, d, b in messages if a in RADAR_IDS]
      if radar:
        self.assertEqual([a for a, _, _ in radar[:2]], [0x2B6, 0x2F6])
        b6, f6 = (h.decode(a, d) for a, d, _ in radar[:2])
        self.assertEqual(b6['MDD_DECEL_CONTROL_REQ'], f6['MDD_DECEL_CONTROL_REQ'])
        counters.append(b6['COUNTER'])
      for message in radar:
        with self.subTest(index=index, message=message):
          self.assertTrue(self.tx(message))
    self.assertGreaterEqual(len(counters), 32)
    self.assertEqual(counters, [i % 16 for i in range(len(counters))])


if __name__ == '__main__':
  unittest.main()
# [psa longitudinal] - END
