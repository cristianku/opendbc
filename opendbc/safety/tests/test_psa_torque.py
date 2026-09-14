# <TEST_ANGLE_START>
"""Torque rollback checks against the compiled hook, including variable factor."""
import unittest

from opendbc.can.packer import CANPacker
from opendbc.car import structs
from opendbc.car.psa.tests.test_angle import AngleHarness, decode_lka
from opendbc.car.psa.values import CAR
from opendbc.safety.tests.libsafety import libsafety_py


class TestPsaTorqueSafety(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(structs.CarParams.SafetyModel.psa, 0)
    self.safety.init_tests()
    self.safety.set_controls_allowed_lateral(True)
    self.packer = CANPacker('psa_aee2010_r3')

  def tx(self, torque, factor=100, **fields):
    values = dict(TORQUE=torque, TORQUE_FACTOR=factor, STATUS=4, unknown2=24)
    values.update(fields)
    addr, data, bus = self.packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)
    return self.safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, data))

  def test_effective_bounds_rates_and_release(self):
    for sign in (-1, 1):
      for last, torque, factor, allowed in ((0, 9, 100, False), (0, 32, 25, True),
                                           (150, 448, 25, True), (150, 444, 25, False),
                                           (150, 151, 100, False), (150, 0, 0, True)):
        with self.subTest(sign=sign, last=last, torque=torque, factor=factor):
          self.setUp()
          self.safety.set_desired_torque_last(sign * last)
          self.safety.set_rt_torque_last(sign * last)
          self.assertEqual(self.tx(sign * torque, factor), allowed)

  def test_permissions_brake_and_wrong_mode(self):
    for fields in ({'DRIVE': 1}, {'LXA_ACTIVATION': 1}, {'SET_ANGLE': 1}):
      self.assertFalse(self.tx(1, **fields))
    self.assertFalse(self.tx(1, factor=0))
    self.assertFalse(self.tx(1, factor=101))
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(False)
    self.assertFalse(self.tx(1))
    self.assertTrue(self.tx(0, 0))
    addr, data, bus = self.packer.make_can_msg('Dat_BSI', 2, {'P013_MainBrake': 1})
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, bus, data))
    self.safety.set_controls_allowed_lateral(True)
    self.assertFalse(self.tx(1))
    self.assertTrue(self.tx(0, 0))

  def test_generated_torque_modes_with_factor_changes_and_raw_driver_override(self):
    for candidate in (CAR.PSA_PEUGEOT_3008, CAR.PSA_CITROEN_C4_SPACETOURER):
      self.setUp()
      h = AngleHarness(candidate, angle_enabled=False)
      h.cc.actuators.torque = 1.0
      for frame in range(250):
        self.safety.set_timer(frame * 10_000)
        if frame == 100:
          h.cc.actuators.torque = 0.01
        if frame == 120:
          h.cc.actuators.torque = -1.0
        raw = 18 if 200 <= frame < 220 else 0
        h.cs.steering['DRIVER_TORQUE'] = raw
        addr, data, bus = self.packer.make_can_msg('STEERING', 0, {'DRIVER_TORQUE': raw})
        self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, bus, data)))
        _, messages = h.step()
        for addr, data, bus in messages:
          if addr == 0x3F2:
            fields = decode_lka(data)
            if raw:
              self.assertEqual(fields.torque, 0)
            self.assertTrue(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, data)),
                            (candidate, frame, data.hex()))

  def test_rejected_shape_does_not_advance_rate_limiter(self):
    self.assertFalse(self.tx(8, DRIVE=1))
    self.assertFalse(self.tx(16))
    self.assertTrue(self.tx(8))


if __name__ == '__main__':
  unittest.main()
# <TEST_ANGLE_START_END>
