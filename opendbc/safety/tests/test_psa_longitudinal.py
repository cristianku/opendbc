# [psa longitudinal] - START
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
          packet = libsafety_py.make_CANPacket(0x56E, 2, b'\x00\x00\x00\x01\x00')
        else:
          packet = libsafety_py.make_CANPacket(0x412, 2, b'\x20' + bytes(7))
        self.assertTrue(self.safety.safety_rx_hook(packet))
        self.assertTrue(self.safety.get_gas_pressed_prev() if pedal == 'gas' else self.safety.get_brake_pressed_prev())
        self.safety.set_controls_allowed(True)
        self.assertFalse(self.tx(self.message(0x2B6, self.gmp)))
        self.assertFalse(self.tx(self.message(0x2B6, self.braking)))
        self.assertFalse(self.tx(self.message(0x2F6, dict(self.display, MDD_DECEL_CONTROL_REQ=1))))
        self.assertTrue(self.tx(self.message(0x2B6, self.neutral)))

  def test_torque_and_deceleration_boundaries(self):
    for field, quantum in (('GMP_WHEEL_TORQUE', 1), ('GMP_POTENTIAL_WHEEL_TORQUE', 4)):
      for torque in (-400-quantum, -400, 1000, 1000+quantum):
        with self.subTest(field=field, torque=torque):
          self.assertEqual(self.tx(self.message(0x2B6, dict(self.gmp, **{field: torque}))), -400 <= torque <= 1000)
    for accel in (-1.05, -1, -0.55, -0.5, -0.45, 2.05):
      with self.subTest(accel=accel):
        self.assertEqual(self.tx(self.message(0x2B6, dict(self.braking, MDD_DESIRED_DECELERATION=accel))), -1 <= accel <= -0.5)

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
