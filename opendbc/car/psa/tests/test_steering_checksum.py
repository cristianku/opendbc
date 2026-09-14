# [steering checksum] - START
"""Checksum regression using recorded 3008 frames, independent of control mode."""
import json
from pathlib import Path
import unittest

from opendbc.can.dbc import DBC
from opendbc.car.psa.psacan import psa_checksum


def recorded_steering_sequences():
  return json.loads(Path(__file__).with_name('steering_alt_recorded.json').read_text())['sequences']


class TestRecordedSteeringChecksum(unittest.TestCase):
  def setUp(self):
    self.sig = DBC('psa_aee2010_r3').addr_to_msg[0x305].sigs['0_CHECKSUM']

  def test_checksum_matches_every_recorded_sample(self):
    for segment, sequence in recorded_steering_sequences().items():
      with self.subTest(segment=segment):
        for _, payload in sequence:
          data = bytes.fromhex(payload)
          self.assertEqual(psa_checksum(0x305, self.sig, bytearray(data)), data[4] >> 4, payload)

  def test_corrupted_protected_bit_changes_checksum_result(self):
    _, payload = recorded_steering_sequences()['00000066--f4919151f0--0'][0]
    for bit in range(40):
      with self.subTest(bit=bit):
        data = bytearray.fromhex(payload)
        data[bit // 8] ^= 1 << (bit % 8)
        received = data[4] >> 4
        self.assertNotEqual(psa_checksum(0x305, self.sig, data), received)


if __name__ == '__main__':
  unittest.main()
# [steering checksum] - END
