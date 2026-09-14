# [torque safety] - START
"""Torque control checks against the compiled hook, including variable factor."""
import unittest
from types import SimpleNamespace

from opendbc.can.packer import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.psa.carcontroller import CarController
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.values import CAR
from opendbc.safety.tests.libsafety import libsafety_py


class TorqueHarness:
  def __init__(self, candidate):
    cp = CarInterface.get_non_essential_params(candidate)
    cp_sp = CarInterface.get_non_essential_params_sp(cp, candidate)
    self.controller = CarController({Bus.main: 'psa_aee2010_r3'}, cp, cp_sp)
    self.controller.model_sm = None
    self.cs = SimpleNamespace(out=structs.CarState(), eps_active=True, eps_state_lka=3,
                              speed_kph=72, is_dat_dira={}, HS2_DYN_MDD_ETAT_2F6={}, steering={'DRIVER_TORQUE': 0})
    self.cs.out.canValid = True
    self.cs.out.vEgo = self.cs.out.vEgoRaw = 20
    self.cs.out.steeringAngleDeg = 12
    self.cc = structs.CarControl()
    self.cc.latActive = True

  def step(self):
    now = (self.controller.frame + 1) * 10_000_000
    return self.controller.update(self.cc.as_reader(), structs.CarControlSP(), self.cs, now)


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
      h = TorqueHarness(candidate)
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
            if raw:
              self.assertEqual((data[3] << 3) | (data[4] >> 5), 0)
            self.assertTrue(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, data)),
                            (candidate, frame, data.hex()))

  def test_rejected_shape_does_not_advance_rate_limiter(self):
    self.assertFalse(self.tx(8, DRIVE=1))
    self.assertFalse(self.tx(16))
    self.assertTrue(self.tx(8))


if __name__ == '__main__':
  unittest.main()
# [torque safety] - END
