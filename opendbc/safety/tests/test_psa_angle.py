# <TEST_ANGLE_START>
import unittest

from opendbc.can.packer import CANPacker
from opendbc.car import structs
from opendbc.safety.tests.libsafety import libsafety_py


def angle_packet(angle=12, active=True, **fields):
  values = {'DRIVE': 1, 'LXA_ACTIVATION': 1, 'TORQUE': 0, 'TORQUE_FACTOR': 100 if active else 0,
            'SET_ANGLE': angle, 'STATUS': 2 if active else 0, 'unknown2': 0, '0_COUNTER': 0}
  values.update(fields)
  data = bytearray(CANPacker('psa_aee2010_r3').make_can_msg('LANE_KEEP_ASSIST', 0, values)[1])
  data[1] &= 15
  data[1] |= ((11 - sum((b >> 4) + (b & 15) for b in data)) & 15) << 4
  return libsafety_py.make_CANPacket(0x3F2, 0, data)


class TestPsaAngleSafety(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.assertEqual(self.safety.set_safety_hooks(structs.CarParams.SafetyModel.psa, 2), 0)
    self.safety.init_tests()
    self.packer = CANPacker('psa_aee2010_r3')
    self.angle_counter = 0
    self.safety.set_controls_allowed(True)
    self.safety.set_controls_allowed_lateral(True)
    self.rx_angle(12)
    self.rx('IS_DAT_DIRA', 0, {'EPS_STATE_LKA': 3})
    for _ in range(6):
      self.rx('Dyn4_FRE', 0, dict.fromkeys(('P263_VehV_VPsvValWhlFrtL', 'P264_VehV_VPsvValWhlFrtR',
                                          'P265_VehV_VPsvValWhlBckL', 'P266_VehV_VPsvValWhlBckR'), 72))
    self.safety.set_desired_angle_last(120)

  def rx(self, name, bus, values):
    address, data, bus = self.packer.make_can_msg(name, bus, values)
    self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, data)))

  def rx_angle(self, angle, bus=0):
    # STEERING_ALT uses prefixed checksum/counter names, encoded explicitly.
    data = bytearray(self.packer.make_can_msg('STEERING_ALT', bus, {'ANGLE': angle, '0_COUNTER': self.angle_counter})[1])
    self.angle_counter = (self.angle_counter + 1) % 16
    data[4] &= 15
    data[4] |= ((11 - sum((b >> 4) + (b & 15) for b in data)) & 15) << 4
    self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x305, bus, data)))

  def test_accepts_neutral_and_valid_active_angle(self):
    self.assertTrue(self.safety.safety_tx_hook(angle_packet(active=False)))
    self.assertTrue(self.safety.safety_tx_hook(angle_packet()))

  def test_rejects_actuation_without_permission(self):
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(False)
    self.assertFalse(self.safety.safety_tx_hook(angle_packet()))
    self.assertTrue(self.safety.safety_tx_hook(angle_packet(active=False)))

  def test_rejects_angle_jump_absolute_limit_and_nonzero_torque(self):
    for kwargs in ({'angle': -20}, {'angle': 90.1}, {'TORQUE': 1}, {'TORQUE_FACTOR': 101},
                   {'LXA_ACTIVATION': 0}, {'DRIVE': 0}, {'STATUS': 1}, {'unknown2': 24}):
      with self.subTest(kwargs=kwargs):
        self.safety.set_desired_angle_last(120)
        self.assertFalse(self.safety.safety_tx_hook(angle_packet(**kwargs)))

  def test_rejects_bad_checksum_and_stale_feedback(self):
    packet = angle_packet()
    packet[0].data[1] ^= 16
    self.assertFalse(self.safety.safety_tx_hook(packet))
    self.safety.set_timer(1_000_001)
    self.assertFalse(self.safety.safety_tx_hook(angle_packet()))

  def test_camera_cannot_overwrite_angle_measurement(self):
    self.rx_angle(-30, bus=2)
    self.assertEqual(self.safety.get_angle_meas_min(), 0)
    self.assertEqual(self.safety.get_angle_meas_max(), 120)

  def test_no_synthetic_driver_feedback(self):
    for address, bus, length in ((0x495, 2, 4), (0x2F5, 0, 7)):
      self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, bytes(length))))

  def test_measured_angle_outside_test_bounds_cannot_activate(self):
    self.rx_angle(200)
    self.safety.set_desired_angle_last(900)
    self.assertFalse(self.safety.safety_tx_hook(angle_packet(angle=90)))

  def test_driver_input_brake_and_eps_fault_release_actuation(self):
    for name, bus, values in (('STEERING', 0, {'DRIVER_TORQUE': 17}),
                              ('Dat_BSI', 2, {'P013_MainBrake': 1}),
                              ('IS_DAT_DIRA', 0, {'EPS_STATE_LKA': 4})):
      with self.subTest(name=name):
        self.setUp()
        self.rx(name, bus, values)
        self.assertFalse(self.safety.safety_tx_hook(angle_packet()))
        self.assertTrue(self.safety.safety_tx_hook(angle_packet(active=False)))

  def test_generated_controller_frames_pass_compiled_safety(self):
    from opendbc.car.psa.tests.test_angle import AngleHarness, decode_lka
    h = AngleHarness()
    # A real frame updates measurement, then the generated command crosses the C hook.
    for frame in range(1600):
      self.safety.set_timer(frame * 10000)
      if frame == 1400:
        h.cc.latActive = False
      if frame == 1500:
        h.cc.latActive = True
      self.rx_angle(h.cs.out.steeringAngleDeg)
      if frame % 10 == 0:
        self.rx('IS_DAT_DIRA', 0, {'EPS_STATE_LKA': 3})
      _, _, message = h.lka()
      self.assertTrue(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(message[0], message[2], message[1])),
                      (frame, message[1].hex()))
      h.cs.out.steeringAngleDeg = decode_lka(message[1]).angle

  def test_required_rx_checks_detect_missing_angle_with_other_inputs_alive(self):
    for frame in range(160):
      self.safety.set_timer(frame * 10000)
      self.rx('HS2_DAT_MDD_CMD_452', 1, {'LONGITUDINAL_REGULATION_TYPE': 3, 'RVV_ACC_ACTIVATION_REQ': 1})
      self.rx('Dyn4_FRE', 0, {})
      self.rx('HS2_DYN_ABR_38D', 0, {})
      self.rx('STEERING', 0, {})
      self.rx('Dat_BSI', 2, {})
      # The 3008 receive check uses DLC 8; the generic DBC definition has DLC 5.
      self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x228, 0, bytes(8))))
      self.rx('IS_DAT_DIRA', 0, {'EPS_STATE_LKA': 3})
      if frame < 20:
        self.rx_angle(12)
      self.safety.safety_tick_current_safety_config()
      if frame == 19:
        self.assertTrue(self.safety.safety_config_valid())
    self.assertFalse(self.safety.safety_config_valid())
    self.assertFalse(self.safety.safety_tx_hook(angle_packet()))

  def test_angle_and_longitudinal_safety_flags_can_coexist(self):
    self.assertEqual(self.safety.set_safety_hooks(structs.CarParams.SafetyModel.psa, 3), 0)
    self.safety.init_tests()
    self.safety.set_controls_allowed_lateral(True)
    self.rx_angle(12)
    self.rx('IS_DAT_DIRA', 0, {'EPS_STATE_LKA': 3})
    self.assertTrue(self.safety.safety_tx_hook(angle_packet()))
    self.assertFalse(self.safety.safety_tx_hook(angle_packet(TORQUE=1)))

  def test_delayed_plant_resynchronizes_after_driver_spike_and_ack_loss(self):
    from opendbc.car.psa.tests.test_angle import AngleHarness
    for event in ('driver', 'ack'):
      with self.subTest(event=event):
        self.setUp()
        h = AngleHarness()
        # Keep the measured wheel at 12 degrees while requests decrease.
        for frame in range(20):
          raw_torque = 17 if event == 'driver' and frame == 10 else 0
          eps_state = 2 if event == 'ack' and 10 <= frame < 13 else 3
          self.safety.set_timer(frame * 10000)
          h.cs.steering['DRIVER_TORQUE'] = raw_torque
          h.cs.eps_active = eps_state == 3
          h.cs.eps_state_lka = eps_state
          self.rx('STEERING', 0, {'DRIVER_TORQUE': raw_torque})
          self.rx('IS_DAT_DIRA', 0, {'EPS_STATE_LKA': eps_state})
          self.rx_angle(12)
          _, values, message = h.lka()
          if frame == 10:
            self.assertEqual((values.factor, values.status, values.angle), (0, 0, 12))
          self.assertTrue(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(message[0], message[2], message[1])),
                          (event, frame, message[1].hex()))

  def test_brief_feedback_gap_releases_and_recovers_while_canvalid_stays_true(self):
    from opendbc.can.parser import CANParser
    from opendbc.car.psa.tests.test_angle import AngleHarness
    h = AngleHarness()
    parser = CANParser('psa_aee2010_r3', [('STEERING_ALT', 100), ('Dyn4_FRE', 50)], 0)
    released = False
    for frame in range(45):
      now = (frame + 1) * 10_000_000
      self.safety.set_timer(now // 1000)
      angle_rx = not 10 <= frame < 21
      frames = [self.packer.make_can_msg('Dyn4_FRE', 0, {})]
      if angle_rx:
        frames.append(self.packer.make_can_msg('STEERING_ALT', 0, {'ANGLE': 12}))
        self.rx_angle(12)
      self.rx('IS_DAT_DIRA', 0, {'EPS_STATE_LKA': 3})
      parser.update([(now, frames)])
      self.assertTrue(parser.can_valid)
      h.cs.angle_feedback_ts = parser.ts_nanos['STEERING_ALT']['ANGLE']
      _, values, message = h.lka(update_angle=False)
      self.assertTrue(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(message[0], message[2], message[1])), frame)
      released |= values.factor == 0
      if frame >= 21:
        self.assertEqual(values.factor, 100)
    self.assertTrue(released)

  def test_corrupt_angle_feedback_releases_and_recovers(self):
    from opendbc.car import Bus
    from opendbc.car.psa.carstate import CarState
    from opendbc.car.psa.tests.test_angle import AngleHarness
    for fault in ('checksum', 'counter'):
      with self.subTest(fault=fault):
        self.setUp()
        h = AngleHarness()
        parser = CarState.get_can_parsers(h.cp, h.controller.CP_SP)[Bus.main]
        _ = parser.vl['STEERING_ALT']  # Match CarState's lazy signal registration.
        released = False
        counter = 0
        for frame in range(45):
          now = (frame + 1) * 10_000_000
          self.safety.set_timer(now // 1000)
          corrupt = 10 <= frame < 25
          if fault != 'counter' or not corrupt:
            counter = (counter + 1) % 16
          data = bytearray(self.packer.make_can_msg('STEERING_ALT', 0, {'ANGLE': 12, '0_COUNTER': counter})[1])
          data[4] &= 15
          data[4] |= ((11 - sum((b >> 4) + (b & 15) for b in data)) & 15) << 4
          if fault == 'checksum' and corrupt:
            data[4] ^= 16
          self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x305, 0, data))
          parser.update([(now, [(0x305, bytes(data), 0)])])
          self.rx('IS_DAT_DIRA', 0, {'EPS_STATE_LKA': 3})
          h.cs.angle_feedback_ts = parser.ts_nanos['STEERING_ALT']['ANGLE']
          _, values, message = h.lka(update_angle=False)
          self.assertTrue(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(message[0], message[2], message[1])),
                          (fault, frame, message[1].hex()))
          released |= values.factor == 0
          if frame >= 25:
            self.assertEqual(values.factor, 100)
        self.assertTrue(released)


if __name__ == '__main__':
  unittest.main()
# <TEST_ANGLE_START_END>
