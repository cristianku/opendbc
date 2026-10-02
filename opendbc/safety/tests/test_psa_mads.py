import unittest

from opendbc.can.packer import CANPacker
from opendbc.car import structs
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.tests.test_longitudinal import LongitudinalHarness
from opendbc.car.psa.values import PSA_LONG_CONTROL
from opendbc.safety.tests.libsafety import libsafety_py


class TestPsaMads(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.packer = CANPacker('psa_aee2010_r3')

  def configure(self, flag, mads=True):
    self.assertEqual(self.safety.set_safety_hooks(structs.CarParams.SafetyModel.psa, flag), 0)
    self.safety.init_tests()
    self.safety.set_mads_params(mads, True, False)

  def rx(self, name, bus, values):
    address, data, bus = self.packer.make_can_msg(name, bus, values)
    self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, data)))

  def cruise(self, mode, engaged=False, bus=1):
    self.rx('HS2_DAT_MDD_CMD_452', bus, {
      'LONGITUDINAL_REGULATION_TYPE': mode, 'RVV_ACC_ACTIVATION_REQ': engaged,
    })

  def test_main_selector_enables_only_lateral_and_main_off_disables_it(self):
    for flag in (0, PSA_LONG_CONTROL):
      for mads in (False, True):
        self.configure(flag, mads)
        for mode, main in ((0, False), (3, True), (2, False), (1, True), (0, False)):
          with self.subTest(flag=flag, mads=mads, mode=mode):
            self.cruise(mode)
            self.assertEqual(self.safety.get_acc_main_on(), main)
            self.assertEqual(self.safety.get_controls_allowed_lateral(), main and mads)
            self.assertFalse(self.safety.get_controls_allowed())
            self.assertFalse(self.safety.get_longitudinal_allowed())

  def test_brake_disengages_until_another_driver_engagement(self):
    for flag in (0, PSA_LONG_CONTROL):
      self.configure(flag)
      self.cruise(0)
      self.cruise(3)
      self.assertTrue(self.safety.get_controls_allowed_lateral())
      self.rx('Dat_BSI', 2, {'P013_MainBrake': 1})
      self.assertFalse(self.safety.get_controls_allowed_lateral())
      self.rx('Dat_BSI', 2, {'P013_MainBrake': 0})
      self.cruise(3)
      self.assertFalse(self.safety.get_controls_allowed_lateral())
      self.cruise(3, engaged=True)
      self.assertTrue(self.safety.get_controls_allowed_lateral())
      self.assertTrue(self.safety.get_controls_allowed())

  def test_other_buses_and_diagnostic_response_cannot_enable_main(self):
    self.configure(PSA_LONG_CONTROL)
    for bus in (0, 2):
      self.cruise(3, bus=bus)
      self.assertFalse(self.safety.get_acc_main_on())
    self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x696, 1, b'\x06\x50\x02\x00\xc8\x00\x14')))
    self.assertFalse(self.safety.get_acc_main_on())
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_recorded_cruise_frames_agree_with_carstate(self):
    # Original bus-1 0x452 frames from the dashcam "bibbia"; timestamps are logMonoTime.
    samples = (
      ('3a--e445e79563--0', 55387601071, '00ff0032f80a', False, False),
      ('3a--e445e79563--0', 56086119612, '03ff0012f809', True, False),
      ('3a--e445e79563--2', 234780466781, '031f827a520f', True, True),
      ('49--a95dde6809--0', 181598317756, '03ff00e2f80c', True, False),
      ('49--a95dde6809--1', 265002533985, '0320822a5202', True, True),
      ('49--a95dde6809--9', 753388557304, '00ff00a2f803', False, False),
    )
    for experimental in (False, True):
      for segment, timestamp, payload, main, engaged in samples:
        with self.subTest(experimental=experimental, segment=segment, timestamp=timestamp):
          h = LongitudinalHarness(experimental=experimental)
          interface = CarInterface(h.controller.CP, h.controller.CP_SP)
          # Register lazily parsed CarState messages before injecting the recorded frame.
          interface.update([(1, [])])
          self.configure(PSA_LONG_CONTROL if experimental else 0)
          data = bytes.fromhex(payload)
          self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x452, 1, data)))
          state, _ = interface.update([(50_000_000, [(0x452, data, 1)])])
          self.assertEqual(state.cruiseState.available, main)
          self.assertEqual(self.safety.get_acc_main_on(), main)
          self.assertEqual(self.safety.get_controls_allowed(), engaged)
          # The longitudinal interface still blocks engagement before radar readiness.
          self.assertEqual(state.cruiseState.enabled, engaged and not experimental)


if __name__ == '__main__':
  unittest.main()
