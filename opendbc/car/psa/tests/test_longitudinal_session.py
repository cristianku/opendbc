# [psa longitudinal] - START
import unittest

from opendbc.car import structs
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.tests.test_longitudinal import LongitudinalHarness, RADAR_IDS


class TestLongitudinalSession(unittest.TestCase):
  # [neutral motion] - START
  def test_confirmed_session_can_move_in_both_profiles(self):
    for experimental in (True, False):
      with self.subTest(experimental=experimental):
        h = LongitudinalHarness(experimental=experimental)
        h.activate()
        h.emission()
        h.cs.out.standstill = False
        h.cs.out.vEgo = 5.0
        for _ in range(20):
          h.step()
        self.assertTrue(h.controller.neutral_radar.active)
        self.assertTrue(any(a in RADAR_IDS for a, _, _ in h.previous))

  def test_moving_before_acceptance_does_not_start_emulation(self):
    for experimental in (True, False):
      with self.subTest(experimental=experimental):
        h = LongitudinalHarness(experimental=experimental)
        h.activate()
        h.cs.out.standstill = False
        _, messages = h.step()
        self.assertFalse(any(a in RADAR_IDS for a, _, _ in messages))
        self.assertFalse(h.controller.neutral_radar.active)
  # [neutral motion] - END

  def test_disengagement_keeps_session_and_reengagement_restores_commands(self):
    h = LongitudinalHarness()
    h.activate()
    h.emission()
    h.cs.out.standstill = False
    h.cs.out.vEgo = 5.0
    h.cc.longActive = False
    _, values = h.emission()
    self.assertTrue(h.controller.neutral_radar.active)
    self.assertIn(0x2B6, values)
    self.assertEqual(values[0x2B6]['WHEEL_TORQUE_REQUEST'], 0)
    h.cc.longActive = True
    _, values = h.emission()
    self.assertEqual(values[0x2B6]['WHEEL_TORQUE_REQUEST'], 1)

  def test_stock_radar_return_during_braking_stops_substitutes_and_reported_request(self):
    h = LongitudinalHarness()
    h.activate()
    h.cc.actuators.accel = -0.75
    h.emission()
    now = h.frame * 10_000_000
    h.controller.neutral_radar.process_can([(now, [(0x2F6, bytes.fromhex('00ff8600f8600f00'), 1)])])
    output, messages = h.step()
    self.assertFalse(any(a in RADAR_IDS or a == 0x6B6 for a, _, _ in messages))
    self.assertEqual(output.accel, 0)
    self.assertEqual(h.controller.neutral_radar.stop_reason, 'stock radar resumed')

  def test_invalid_can_cancels_actuation_before_echo_grace_expires(self):
    h = LongitudinalHarness()
    h.activate()
    h.emission()
    h.cs.out.canValid = False
    output, values = h.emission()
    self.assertEqual(output.accel, 0)
    self.assertEqual(values[0x2B6]['POTENTIAL_WHEEL_TORQUE_REQUEST'], 0)
    self.assertEqual(values[0x2F6]['MDD_DECEL_CONTROL_REQ'], 0)

  def test_tester_timeout_during_braking_removes_substitutes(self):
    h = LongitudinalHarness()
    h.activate()
    h.cc.actuators.accel = -0.75
    h.emission()
    # Keep genuine bus RX and TX receipts alive, but no diagnostic replies.
    for frame in range(h.frame, h.frame + 210):
      now = frame * 10_000_000
      h.controller.neutral_radar.process_can([(now, [(0x212, bytes(8), 1)] +
                                                   [(a, d, 129) for a, d, _ in h.previous if a in RADAR_IDS])])
      h.controller.frame = frame
      output, h.previous = h.controller.update(h.cc.as_reader(), structs.CarControlSP(), h.cs, now)
    self.assertEqual(h.controller.neutral_radar.stop_reason, 'TesterPresent response timeout')
    self.assertEqual(output.accel, 0)
    self.assertFalse(any(a in RADAR_IDS for a, _, _ in h.previous))

  def test_interface_exposes_session_fault_in_carstate(self):
    h = LongitudinalHarness()
    interface = CarInterface(h.controller.CP, h.controller.CP_SP)
    # The actual interface/CarState path must export the session failure, independent of stock ACC faults.
    interface.CC.neutral_radar.stop('TesterPresent response timeout')
    state, _ = interface.update([(20_000_000_000, [])])
    self.assertTrue(state.accFaulted)
    self.assertFalse(state.cruiseState.available)
    self.assertFalse(state.cruiseState.enabled)
    self.assertTrue(interface.CS.out.accFaulted)

  def test_interface_does_not_allow_engagement_while_waiting_for_session(self):
    h = LongitudinalHarness()
    interface = CarInterface(h.controller.CP, h.controller.CP_SP)
    state, _ = interface.update([(1, [])])
    self.assertFalse(state.cruiseState.available)


if __name__ == '__main__':
  unittest.main()
# [psa longitudinal] - END
