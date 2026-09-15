import unittest
from types import SimpleNamespace

from opendbc.car import Bus, structs
from opendbc.car.psa.carcontroller import CarController
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.values import CAR


RADAR_IDS = {0x2B6, 0x2F6, 0x4F6, 0x796}


# [radar handover timing] - START
class TestRadarHandoverTiming(unittest.TestCase):
  def setUp(self):
    cp = CarInterface.get_params(CAR.PSA_PEUGEOT_3008, {0: {}, 1: {}, 2: {}}, [], True, False, False)
    cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.PSA_PEUGEOT_3008)
    self.controller = CarController({Bus.main: 'psa_aee2010_r3'}, cp, cp_sp)
    self.cs = SimpleNamespace(eps_active=False, speed_kph=0.0, out=structs.CarState())
    self.cs.out.standstill = True
    self.cs.out.canValid = True

    stock = [
      (0x2B6, bytes.fromhex('fe0000020000030a'), 1),
      (0x2F6, bytes.fromhex('00ff8600f8600f00'), 1),
      (0x4F6, bytes.fromhex('fffe5ffe00'), 1),
      (0x796, bytes(8), 1),
    ]
    self.controller.process_radar_can([(10_000_000_000, stock)])
    self.controller.frame = 1000
    _, sent = self.controller.update(structs.CarControl().as_reader(), structs.CarControlSP(), self.cs, 10_000_000_000)
    self.assertIn((0x6B6, b'\x02\x10\x02', 1), sent)

  def step(self, frame, nanos):
    self.controller.frame = frame
    _, sent = self.controller.update(structs.CarControl().as_reader(), structs.CarControlSP(), self.cs, nanos)
    return [m for m in sent if m[0] in RADAR_IDS]

  def test_neutral_bridge_starts_on_first_expired_stock_period_before_positive_response(self):
    self.assertEqual(self.step(1001, 10_010_000_000), [])
    messages = self.step(1002, 10_020_000_000)

    self.assertFalse(self.controller.radar_active)
    self.assertEqual([m[0] for m in messages], [0x2B6, 0x2F6])
    self.assertEqual(messages[0][1][7] >> 4, 1)
    self.assertEqual(messages[1][1][6] >> 4, 1)
    self.assertEqual(messages[0][1][:6], bytes.fromhex('fe0000020000'))
    self.assertEqual(messages[1][1][:6], bytes.fromhex('00ff8600f860'))

  def test_positive_response_keeps_existing_message_phase_and_counter(self):
    for frame in (1002, 1004, 1006):
      messages = self.step(frame, frame * 10_000_000)
      self.assertEqual([m[0] for m in messages], [0x2B6, 0x2F6])

    self.controller.process_radar_can([(10_067_000_000, [(0x696, bytes.fromhex('06500200c80014'), 1)])])
    self.assertEqual(self.step(1007, 10_070_000_000), [])
    self.assertTrue(self.controller.radar_active)

    messages = self.step(1008, 10_080_000_000)
    self.assertEqual([m[0] for m in messages], [0x2B6, 0x2F6])
    self.assertEqual(messages[0][1][7] >> 4, 4)
    self.assertEqual(messages[1][1][6] >> 4, 4)
# [radar handover timing] - END


if __name__ == '__main__':
  unittest.main()
