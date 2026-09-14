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

  def send(self, stock_payload=None, bus=2):
    now_nanos = (self.controller.frame + 1) * 10_000_000
    frames = [] if stock_payload is None else [(0x3F2, bytes.fromhex(stock_payload), bus)]
    self.interface.update([(now_nanos, frames)])
    _, messages = self.controller.update(self.cc.as_reader(), self.cc_sp, self.interface.CS, now_nanos)
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
    self.assertEqual(self.send()[1], bytes.fromhex('000018000c140000'))
    self.cc.latActive = False
    self.assertEqual(self.send()[1], bytes.fromhex('00000e0008000000'))


if __name__ == '__main__':
  unittest.main()
# [inactive lka] - END
