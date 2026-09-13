# [psa longitudinal] - START
import unittest

from opendbc.car import DT_CTRL, structs
from opendbc.car.psa.carcontroller import ARTIV_PROGRAMMING_WAIT
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.tests.test_longitudinal import LongitudinalHarness, RADAR_IDS


class TestLongitudinalSession(unittest.TestCase):
  # [radar optin] - START
  def test_confirmed_session_can_move_with_cruise_engaged_or_disengaged(self):
    for engaged in (True, False):
      with self.subTest(engaged=engaged):
        h = LongitudinalHarness()
        h.cc.enabled = engaged
        h.cc.longActive = engaged
        h.activate()
        h.emission()
        h.cs.out.standstill = False
        h.cs.out.vEgo = 5.0
        for _ in range(20):
          h.step()
        self.assertTrue(h.controller.radar_active)
        self.assertTrue(any(a in RADAR_IDS for a, _, _ in h.previous))

  def test_session_can_start_while_moving_after_radar_acceptance(self):
    for engaged in (True, False):
      with self.subTest(engaged=engaged):
        h = LongitudinalHarness()
        h.cc.enabled = engaged
        h.cc.longActive = engaged
        h.cs.out.standstill = False
        h.cs.out.vEgo = 5.0
        h.activate()
        _, messages = h.step()
        self.assertTrue(any(a in RADAR_IDS for a, _, _ in messages))
        self.assertTrue(h.controller.radar_active)
        self.assertIsNone(h.controller.radar_stop_reason)

  def test_moving_start_waits_for_valid_can_and_requests_only_once(self):
    h = LongitudinalHarness()
    h.frame = 0
    h.cs.out.standstill = False
    h.cs.out.vEgo = 5.0
    wait_frames = int(ARTIV_PROGRAMMING_WAIT / DT_CTRL)
    for _ in range(wait_frames):
      _, messages = h.step()
      self.assertFalse(any(a == 0x6B6 for a, _, _ in messages))
    # Invalid CAN resets the wait even though the first interval has elapsed.
    h.cs.out.canValid = False
    _, messages = h.step()
    self.assertFalse(any(a == 0x6B6 for a, _, _ in messages))
    h.cs.out.canValid = True
    for _ in range(wait_frames - 1):
      _, messages = h.step()
      self.assertFalse(any(a == 0x6B6 for a, _, _ in messages))
    _, messages = h.step()
    self.assertIn((0x6B6, b'\x02\x10\x02', 1), messages)
    self.assertFalse(any(a in RADAR_IDS for a, _, _ in messages))
    # Without a positive response there must be no emulation or repeated request.
    for _ in range(120):
      _, messages = h.step()
      self.assertFalse(any(a in RADAR_IDS or a == 0x6B6 for a, _, _ in messages))
    self.assertEqual(h.controller.radar_stop_reason, 'no confirmed silent radar within 1 s')
  # [radar optin] - END

  def test_disengagement_keeps_session_and_reengagement_restores_commands(self):
    h = LongitudinalHarness()
    h.activate()
    h.emission()
    h.cs.out.standstill = False
    h.cs.out.vEgo = 5.0
    h.cc.longActive = False
    _, values = h.emission()
    self.assertTrue(h.controller.radar_active)
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
    h.controller.process_radar_can([(now, [(0x2F6, bytes.fromhex('00ff8600f8600f00'), 1)])])
    output, messages = h.step()
    self.assertFalse(any(a in RADAR_IDS or a == 0x6B6 for a, _, _ in messages))
    self.assertEqual(output.accel, 0)
    self.assertEqual(h.controller.radar_stop_reason, 'stock radar resumed')

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
      h.controller.process_radar_can([(now, [(0x212, bytes(8), 1)] +
                                                   [(a, d, 129) for a, d, _ in h.previous if a in RADAR_IDS])])
      h.controller.frame = frame
      output, h.previous = h.controller.update(h.cc.as_reader(), structs.CarControlSP(), h.cs, now)
    self.assertEqual(h.controller.radar_stop_reason, 'TesterPresent response timeout')
    self.assertEqual(output.accel, 0)
    self.assertFalse(any(a in RADAR_IDS for a, _, _ in h.previous))

  def test_interface_exposes_session_fault_in_carstate(self):
    h = LongitudinalHarness()
    interface = CarInterface(h.controller.CP, h.controller.CP_SP)
    # The actual interface/CarState path must export the session failure, independent of stock ACC faults.
    interface.CC._stop_radar_session('TesterPresent response timeout')
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

  def test_cruise_main_follows_selector_independently_of_radar_session(self):
    # A radar-ready transition must not look like a cruise-main button press to MADS.
    for experimental in (False, True):
      h = LongitudinalHarness(experimental=experimental)
      interface = CarInterface(h.controller.CP, h.controller.CP_SP)
      frame = 0
      for mode, available in ((0, False), (3, True), (2, False), (1, True), (0, False)):
        for radar_active in (False, True, False):
          with self.subTest(experimental=experimental, mode=mode, radar_active=radar_active):
            interface.CC.radar_active = radar_active
            msg = h.packer.make_can_msg('HS2_DAT_MDD_CMD_452', 1, {
              'LONGITUDINAL_REGULATION_TYPE': mode, 'RVV_ACC_ACTIVATION_REQ': int(available),
            })
            frame += 1
            state, _ = interface.update([(frame * 50_000_000, [msg])])
            self.assertEqual(state.cruiseState.available, available)
            self.assertEqual(state.cruiseState.enabled, available and (not experimental or radar_active))


if __name__ == '__main__':
  unittest.main()
# [psa longitudinal] - END
