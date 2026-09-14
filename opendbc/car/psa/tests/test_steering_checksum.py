# <TEST_ANGLE_START>
"""Golden RX frames from the 3008, independent of our synthetic CAN encoder."""
import json
from pathlib import Path
import unittest

from opendbc.car import Bus
from opendbc.car.psa.carstate import CarState
from opendbc.car.psa.psacan import psa_checksum
from opendbc.car.psa.tests.test_angle import AngleHarness


def recorded_steering_sequences():
  return json.loads(Path(__file__).with_name('steering_alt_recorded.json').read_text())['sequences']


class TestRecordedSteeringChecksum(unittest.TestCase):
  def test_real_checksum_and_parser_accept_every_recorded_sample(self):
    for segment, sequence in recorded_steering_sequences().items():
      with self.subTest(segment=segment):
        h = AngleHarness()
        parser = CarState.get_can_parsers(h.cp, h.controller.CP_SP)[Bus.main]
        sig = parser.dbc.addr_to_msg[0x305].sigs['0_CHECKSUM']
        for ts, payload in sequence:
          data = bytes.fromhex(payload)
          self.assertEqual(psa_checksum(0x305, sig, bytearray(data)), data[4] >> 4, payload)
          parser.update([(ts, [(0x305, data, 0)])])
          self.assertEqual(parser.ts_nanos['STEERING_ALT']['ANGLE'], ts, payload)
          self.assertAlmostEqual(parser.vl['STEERING_ALT']['ANGLE'], int.from_bytes(data[:2], signed=True) / 10)
        self.assertTrue(parser.can_valid)

  def test_corrupted_protected_bit_does_not_update_measurement(self):
    ts, payload = recorded_steering_sequences()['00000066--f4919151f0--0'][0]
    for bit in range(40):
      h = AngleHarness()
      parser = CarState.get_can_parsers(h.cp, h.controller.CP_SP)[Bus.main]
      original = bytes.fromhex(payload)
      parser.update([(ts, [(0x305, original, 0)])])
      data = bytearray(original)
      data[bit // 8] ^= 1 << (bit % 8)
      parser.update([(ts + 10_000_000, [(0x305, bytes(data), 0)])])
      self.assertEqual(parser.ts_nanos['STEERING_ALT']['ANGLE'], ts, bit)


if __name__ == '__main__':
  unittest.main()
# <TEST_ANGLE_START_END>
