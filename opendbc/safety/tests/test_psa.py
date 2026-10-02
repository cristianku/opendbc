#!/usr/bin/env python3
import unittest
# [psa safety] - START
from unittest.mock import patch
# [psa safety] - END

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety

LANE_KEEP_ASSIST = 0x3F2
IS_DAT_DIRA = 0x495
STEERING = 0x2F5
HS2_DYN_MDD_ETAT_2F6 = 0x2F6
REQ_DIAG_ARTIV = 0x6B6
HS2_DAT_MDD_CMD_452 = 0x452
HS2_SUPV_ARTIV_796 = 0x796
HS2_DAT_ARTIV_V2_4F6 = 0x4F6
HS2_DYN1_MDD_ETAT_2B6 = 0x2B6


# [psa safety] - START
class TestPsaSafetyBase(common.CarSafetyTest, common.TorqueSteeringSafetyTestBase, common.VehicleSpeedSafetyTest):
  RELAY_MALFUNCTION_ADDRS = {0: (LANE_KEEP_ASSIST,)}
  FWD_BLACKLISTED_ADDRS = {2: [LANE_KEEP_ASSIST]}
  TX_MSGS = [
    [LANE_KEEP_ASSIST, 0],
    [IS_DAT_DIRA, 2],
    [STEERING, 0],
    [HS2_DYN_MDD_ETAT_2F6, 1],
    [REQ_DIAG_ARTIV, 1],
    [HS2_DAT_MDD_CMD_452, 1],
    [HS2_SUPV_ARTIV_796, 1],
    [HS2_DAT_ARTIV_V2_4F6, 1],
    [HS2_DYN1_MDD_ETAT_2B6, 1],
  ]

  MAIN_BUS = 0
  ADAS_BUS = 1
  CAM_BUS = 2

  MAX_RATE_UP = 8
  MAX_RATE_DOWN = 38
  MAX_TORQUE_LOOKUP = ([0], [150])

  def setUp(self):
    self.packer = CANPackerSafety("psa_aee2010_r3")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.psa, 0)
    self.safety.init_tests()

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"TORQUE": torque, "TORQUE_FACTOR": 100 if steer_req and torque else 0,
              "STATUS": 4, "unknown2": 24}
    return self.packer.make_can_msg_safety("LANE_KEEP_ASSIST", self.MAIN_BUS, values)

  def _driver_torque_msg(self, raw):
    return self.packer.make_can_msg_safety("STEERING", self.MAIN_BUS, {"DRIVER_TORQUE": raw})

  def _pcm_status_msg(self, enable):
    values = {"RVV_ACC_ACTIVATION_REQ": enable, "LONGITUDINAL_REGULATION_TYPE": 3 if enable else 0}
    return self.packer.make_can_msg_safety("HS2_DAT_MDD_CMD_452", self.ADAS_BUS, values)

  def _speed_msg(self, speed):
    values = {name: speed * 3.6 for name in ("P263_VehV_VPsvValWhlFrtL", "P264_VehV_VPsvValWhlFrtR",
                                           "P265_VehV_VPsvValWhlBckL", "P266_VehV_VPsvValWhlBckR")}
    return self.packer.make_can_msg_safety("Dyn4_FRE", self.MAIN_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"P013_MainBrake": brake}
    return self.packer.make_can_msg_safety("Dat_BSI", self.CAM_BUS, values)

  def _user_gas_msg(self, gas):
    return self.packer.make_can_msg_safety("DRIVER", self.CAM_BUS, {"GAS_PEDAL": gas})

  def test_driver_override_threshold_and_zero_release(self):
    # The 3008 treats raw driver torque * 3 above 50 as an immediate override.
    for raw in (-18, -17, -16, 0, 16, 17, 18):
      for torque in (-8, 8):
        with self.subTest(raw=raw, torque=torque):
          self.setUp()
          for _ in range(common.MAX_SAMPLE_VALS):
            self.assertTrue(self._rx(self._driver_torque_msg(raw)))
          self.assertEqual(self.safety.get_torque_driver_min(), raw * 3)
          self.assertEqual(self.safety.get_torque_driver_max(), raw * 3)
          self.safety.set_controls_allowed(True)
          self.assertEqual(self._tx(self._torque_cmd_msg(torque)), abs(raw) <= 16)
          self.assertTrue(self._tx(self._torque_cmd_msg(0)))

  def test_driver_torque_measurements_reset_on_safety_init(self):
    for _ in range(common.MAX_SAMPLE_VALS):
      self.assertTrue(self._rx(self._driver_torque_msg(16)))
    self.assertEqual(self.safety.get_torque_driver_min(), 48)
    self.assertEqual(self.safety.get_torque_driver_max(), 48)
    self._reset_safety_hooks()
    self.assertEqual(self.safety.get_torque_driver_min(), 0)
    self.assertEqual(self.safety.get_torque_driver_max(), 0)

  def test_torque_decrease_limit_and_immediate_zero_release(self):
    for sign in (-1, 1):
      for next_torque, allowed in ((112, True), (111, False), (0, True)):
        with self.subTest(sign=sign, next_torque=next_torque):
          self.setUp()
          self.safety.set_controls_allowed(True)
          self._set_prev_torque(sign * 150)
          self.assertEqual(self._tx(self._torque_cmd_msg(sign * next_torque)), allowed)

  def test_tx_hook_on_wrong_safety_mode(self):
    from opendbc.safety.tests.test_elm327 import TestElm327

    # ELM327 scans 0x600..0x7ff, including PSA's non-actuating radar-health frame.
    # Keep every other cross-mode check; validate that exact overlap below.
    other_diagnostics = [msg for msg in TestElm327.TX_MSGS if msg != [HS2_SUPV_ARTIV_796, self.ADAS_BUS]]
    with patch.object(TestElm327, 'TX_MSGS', other_diagnostics):
      super().test_tx_hook_on_wrong_safety_mode()

  def test_radar_health_overlap_requires_exact_bus_and_length(self):
    for controls in (False, True):
      self.safety.set_controls_allowed(controls)
      for bus in range(4):
        for length in range(9):
          self.assertEqual(self._tx(common.make_msg(bus, HS2_SUPV_ARTIV_796, length)),
                           bus == self.ADAS_BUS and length == 8)
      # The diagnostic-range overlap does not authorize arbitrary radar UDS data.
      self.assertFalse(self._tx(common.make_msg(self.ADAS_BUS, REQ_DIAG_ARTIV)))

  def test_rx_hook(self):
    # The measured wheel-speed source has no checksum. Keep coverage for the
    # separately checked 0x38d message rather than corrupting Dyn4_FRE data.
    for _ in range(10):
      self.assertTrue(self._rx(self._speed_msg(0)))
    for _ in range(10):
      msg = self.packer.make_can_msg_safety("HS2_DYN_ABR_38D", self.MAIN_BUS, {"VITESSE_VEHICULE_ROUES": 0})
      self.assertTrue(self._rx(msg))
    msg[0].data[5] ^= 1
    self.assertFalse(self._rx(msg))

    # cruise
    for _ in range(10):
      self.assertTrue(self._rx(self._pcm_status_msg(0)))
    msg = self._pcm_status_msg(0)
    # invalidate checksum
    msg[0].data[5] = 0x00
    self.assertFalse(self._rx(msg))
    msg = self._pcm_status_msg(0)
    # write to unused payload byte
    msg[0].data[6] = 0xAB
    self.assertTrue(self._rx(msg))

  # [artiv probe] - START
  def test_artiv_neutral_messages(self):
    from opendbc.can.packer import CANPacker
    from opendbc.car.psa import psacan

    packer = CANPacker('psa_aee2010_r3')
    for controls_allowed in (False, True):
      self.safety.set_controls_allowed(controls_allowed)
      for counter in range(16):
        messages = [
          psacan.create_HS2_DYN1_MDD_ETAT_2B6(
            packer, self.ADAS_BUS, mdd_desired_deceleration=2.05, potential_wheel_torque_request=0,
            min_time_for_desired_gear=0, gmp_potential_wheel_torque=-4000, acc_status=2, gmp_wheel_torque=-4000,
            wheel_torque_request=0, auto_braking_status=3, mdd_decel_type=0, mdd_decel_control_req=0,
            gear_type=counter & 1, prefill_request=0, counter=counter,
          ),
          psacan.create_HS2_DYN_MDD_ETAT_2F6(
            packer, self.ADAS_BUS, target_detected=0, request_takeover=0, blind_sensor=0,
            req_visual_coll_alert_arc=0, req_audio_coll_alert_arc=0, req_haptic_coll_alert_arc=0,
            inter_vehicle_distance=255.5, arc_status=6, auto_braking_in_progress=0, aeb_enabled=0,
            drive_away_request=0, display_intervehicle_time=6.2, mdd_decel_control_req=0,
            auto_braking_status=3, counter=counter, target_position=0,
          ),
          psacan.create_HS2_DAT_ARTIV_V2_4F6(
            packer, self.ADAS_BUS, time_gap=25.5, distance_gap=254, relative_speed=93.8,
            artiv_sensor_state=2, target_detected=0, artiv_target_change_info=0, traffic_direction=0,
          ),
          psacan.create_HS2_SUPV_ARTIV_796(
            packer, self.ADAS_BUS, fault_code=0, status_no_config=0, status_partial_wakeup_gmp=0, uce_electr_state=0,
          ),
        ]
        for address, data, bus in messages:
          self.assertTrue(self._tx(libsafety_py.make_CANPacket(address, bus, data)))

  def test_artiv_short_tester_present(self):
    for controls_allowed in (False, True):
      self.safety.set_controls_allowed(controls_allowed)
      for bus in range(3):
        for subfunction in range(256):
          dat = bytes((2, 0x3E, subfunction))
          self.assertEqual(self._tx(common.make_msg(bus, REQ_DIAG_ARTIV, dat=dat)),
                           bus == self.ADAS_BUS and subfunction == 0)

    for dat in (b"\x02\x10\x03", b"\x02\x27\x01",
                b"\x01\x3E\x00", b"\x02\x3E", b"\x02\x3E\x00\x00"):
      self.assertFalse(self._tx(common.make_msg(self.ADAS_BUS, REQ_DIAG_ARTIV, dat=dat)), dat.hex())

  def test_artiv_short_programming_session(self):
    for controls_allowed in (False, True):
      self.safety.set_controls_allowed(controls_allowed)
      for bus in range(3):
        for subfunction in range(256):
          dat = bytes((2, 0x10, subfunction))
          self.assertEqual(self._tx(common.make_msg(bus, REQ_DIAG_ARTIV, dat=dat)),
                           bus == self.ADAS_BUS and subfunction == 2)
    for dat in (b"\x01\x10\x02", b"\x02\x10", b"\x02\x10\x02\x00"):
      self.assertFalse(self._tx(common.make_msg(self.ADAS_BUS, REQ_DIAG_ARTIV, dat=dat)), dat.hex())
  # [artiv probe] - END

  def test_artiv_diagnostics(self):
    allowed = (
      b"\x02\x10\x02\x00\x00\x00\x00\x00",  # programming session
      b"\x02\x3E\x00\x00\x00\x00\x00\x00",  # TesterPresent with response
      b"\x02\x3E\x80\x00\x00\x00\x00\x00",  # TesterPresent, suppress positive response
    )
    for dat in allowed:
      self.assertTrue(self._tx(common.make_msg(self.ADAS_BUS, REQ_DIAG_ARTIV, dat=dat)), dat.hex())

    blocked = (
      b"\x02\x10\x01\x00\x00\x00\x00\x00",  # default session
      b"\x02\x10\x03\x00\x00\x00\x00\x00",  # extended session
      b"\x02\x27\x01\x00\x00\x00\x00\x00",  # SecurityAccess
      b"\x02\x10\x02\x80\x00\x00\x00\x00",  # non-zero padding
    )
    for dat in blocked:
      self.assertFalse(self._tx(common.make_msg(self.ADAS_BUS, REQ_DIAG_ARTIV, dat=dat)), dat.hex())

# [psa safety] - END


class TestPsaStockSafety(TestPsaSafetyBase):

  def setUp(self):
    self.packer = CANPackerSafety("psa_aee2010_r3")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.psa, 0)
    self.safety.init_tests()


if __name__ == "__main__":
    unittest.main()
