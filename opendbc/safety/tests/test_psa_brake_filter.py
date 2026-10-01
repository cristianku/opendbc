# [brake filter] - START
import unittest

from opendbc.car import structs
from opendbc.car.psa.tests.test_longitudinal import LongitudinalHarness, RADAR_IDS
from opendbc.car.psa.values import PSA_LONG_CONTROL
from opendbc.safety.tests.libsafety import libsafety_py
# [brake filter] - END


# [brake filter] - START
class TestPsaBrakeFilterSafety(unittest.TestCase):
  def test_filtered_brake_ramp_override_and_release_pass_real_safety(self):
    safety = libsafety_py.libsafety
    self.assertEqual(safety.set_safety_hooks(structs.CarParams.SafetyModel.psa, PSA_LONG_CONTROL), 0)
    safety.init_tests()
    safety.set_controls_allowed(True)
    h = LongitudinalHarness()
    stock = [h.packer.make_can_msg(name, 1, {}) for name in ('HS2_DYN_MDD_ETAT_2F6', 'HS2_DAT_ARTIV_V2_4F6')]
    h.controller.process_radar_can([(10_000_000_000, stock)])
    h.activate()
    counters = []
    first_braking = []
    for accel, gas, enabled, cycles in ((-0.8, False, True, 80), (-0.8, True, True, 15),
                                      (-0.8, False, True, 80), (-0.1, False, True, 20),
                                      (0.0, False, True, 20), (0.1, False, True, 20),
                                      (-0.8, False, False, 20)):
      h.cc.actuators.accel = accel
      h.cs.out.gasPressed = gas
      h.cc.enabled = enabled
      h.cc.longActive = enabled and not gas
      safety.safety_rx_hook(libsafety_py.make_CANPacket(0x228, 0, bytes([0, 0, int(gas), 0, 0, 0, 0, 0])))
      safety.set_controls_allowed(enabled)
      emitted = 0
      for _ in range(cycles):
        _, messages = h.step()
        for address, data, bus in messages:
          if address in RADAR_IDS:
            self.assertTrue(safety.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, data)), (accel, gas, enabled, address))
          if address == 0x2B6:
            emitted += 1
            values = h.decode(address, data)
            counters.append(values['COUNTER'])
            if gas or not enabled:
              self.assertEqual(values['MDD_DECEL_CONTROL_REQ'], 0)
              self.assertEqual(values['MDD_DESIRED_DECELERATION'], 2.05)
            elif accel <= 0.0:
              self.assertEqual(values['MDD_DECEL_CONTROL_REQ'], 1)
              self.assertGreaterEqual(values['MDD_DESIRED_DECELERATION'], -2.0)
              self.assertLessEqual(values['MDD_DESIRED_DECELERATION'], 0.0)
              if accel == -0.8 and emitted == 1:
                first_braking.append(values['MDD_DESIRED_DECELERATION'])
      self.assertGreater(emitted, 0)
    self.assertEqual(len(first_braking), 2)
    self.assertTrue(all(-0.2 < value <= 0.0 for value in first_braking))
    self.assertTrue(all(following == (previous + 1) % 16 for previous, following in zip(counters, counters[1:])))
# [brake filter] - END


# [brake filter] - START
if __name__ == '__main__':
  unittest.main()
# [brake filter] - END
