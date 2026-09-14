# [inactive lka] - START
import unittest

from opendbc.car import Bus, structs
from opendbc.car.psa.carcontroller import CarController
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.values import CAR


class TestInactiveLka(unittest.TestCase):
  def setUp(self):
    cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_3008)
    cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.PSA_PEUGEOT_3008)
    self.interface = CarInterface(cp, cp_sp)
    self.controller = CarController({Bus.main: 'psa_aee2010_r3'}, cp, cp_sp)
    self.controller.model_sm = None
    self.cc = structs.CarControl()
    self.cc_sp = structs.CarControlSP()
    self.interface.update([(1, [])])

  def send(self, stock_payload=None, bus=2, *, speed_kph=None, eps_state=None):
    now_nanos = (self.controller.frame + 1) * 10_000_000
    frames = [] if stock_payload is None else [(0x3F2, bytes.fromhex(stock_payload), bus)]
    if speed_kph is not None:
      frames.append(self.controller.packer.make_can_msg('Dyn4_FRE', 0, {
        'P263_VehV_VPsvValWhlFrtL': speed_kph,
        'P264_VehV_VPsvValWhlFrtR': speed_kph,
        'P265_VehV_VPsvValWhlBckL': speed_kph,
        'P266_VehV_VPsvValWhlBckR': speed_kph,
      }))
    if eps_state is not None:
      frames.append(self.controller.packer.make_can_msg('IS_DAT_DIRA', 0, {'EPS_STATE_LKA': eps_state}))
    self.interface.update([(now_nanos, frames)])
    _, messages = self.controller.update(self.cc.as_reader(), self.cc_sp, self.interface.CS, now_nanos)
    self.last_messages = messages
    self.controller.frame += 4
    return next(msg for msg in messages if msg[0] == 0x3F2)

  def test_inactive_lka_tracks_camera_byte_without_copying_torque_or_fault_status(self):
    # Route 58 inactive camera payload, then a varying byte and a fault frame.
    for stock, expected in (
      ('00000e0008000000', '00000e0008000000'),
      ('0000050008000000', '0000050008000000'),
      ('0000110094720000', '0000110008000000'),
      ('0000000008000000', '0000000008000000'),
    ):
      with self.subTest(stock=stock):
        msg = self.send(stock)
        self.assertEqual((msg[0], msg[2]), (0x3F2, 0))
        self.assertEqual(msg[1], bytes.fromhex(expected))

  def test_no_camera_sample_preserves_previous_inactive_payload(self):
    self.assertEqual(self.send()[1], bytes.fromhex('0000180008000000'))

  def test_between_camera_samples_keeps_last_received_byte(self):
    self.send('00000e0008000000')
    self.assertEqual(self.send()[1], bytes.fromhex('00000e0008000000'))

  def test_main_bus_lka_cannot_replace_camera_byte(self):
    self.send('00000e0008000000')
    self.assertEqual(self.send('0000630008000000', bus=0)[1], bytes.fromhex('00000e0008000000'))

  def test_active_lka_keeps_existing_activation_payload_then_restores_camera_byte(self):
    self.send('00000e0008000000')
    self.cc.latActive = True
    # EPS is not active yet: preserve the existing activation ladder and factor.
    self.assertEqual(self.send(speed_kph=60)[1], bytes.fromhex('000018000c140000'))
    self.cc.latActive = False
    self.assertEqual(self.send()[1], bytes.fromhex('00000e0008000000'))

  def test_low_speed_blocks_activation_and_expired_rearm_even_if_lat_active_is_stale(self):
    self.cc.latActive = True
    self.cc.actuators.torque = 1.0
    self.send('0000090008000000')
    for speed_kph in (0, 30, 50.9, 51):
      for eps_state in (0, 3):
        with self.subTest(speed_kph=speed_kph, eps_state=eps_state):
          self.controller.frame = 1300
          self.controller.status = 4
          self.controller.eps_activation_frame = 5
          self.controller.deactivation_in_progress = True
          self.controller.apply_torque_factor = 80
          self.controller.apply_torque_scaled_last = 100
          self.controller.steering_hold_counter = self.controller.next_steering_hold
          for _ in range(3):
            msg = self.send(speed_kph=speed_kph, eps_state=eps_state)
            self.assertEqual(msg[1], bytes.fromhex('0000090008000000'))
            self.assertEqual(self.controller.eps_activation_frame, 0)
            self.assertFalse(self.controller.deactivation_in_progress)
            self.assertEqual(self.controller.apply_torque_scaled_last, 0)
            self.assertFalse(self.controller.latActiveLast)
            self.assertFalse(any(m[0] == 0x495 for m in self.last_messages))

  def test_crossing_speed_threshold_starts_a_fresh_eps_cycle(self):
    self.send('0000090008000000')
    self.cc.latActive = True
    self.send(speed_kph=60, eps_state=0)
    self.send(speed_kph=60, eps_state=3)
    self.assertGreater(self.controller.eps_activation_frame, 0)

    self.assertEqual(self.send(speed_kph=50.9, eps_state=3)[1], bytes.fromhex('0000090008000000'))
    self.assertEqual(self.controller.eps_activation_frame, 0)
    self.controller.frame += 1500  # Time below threshold must not count towards rearm.
    self.assertEqual(self.send(speed_kph=51, eps_state=0)[1], bytes.fromhex('0000090008000000'))

    self.assertEqual(self.send(speed_kph=51.01, eps_state=0)[1], bytes.fromhex('000018000c140000'))
    start_frame = self.controller.frame
    self.send(speed_kph=60, eps_state=3)
    self.assertEqual(self.controller.eps_activation_frame, start_frame)
    self.controller.frame = start_frame + self.controller.eps_rearm_frames - self.controller.params.STEER_STEP
    self.send(speed_kph=60, eps_state=3)
    self.assertEqual(self.controller.status, 4)
    self.send(speed_kph=60, eps_state=3)
    self.assertEqual(self.controller.status, 2)
    self.assertTrue(self.controller.deactivation_in_progress)


if __name__ == '__main__':
  unittest.main()
# [inactive lka] - END
