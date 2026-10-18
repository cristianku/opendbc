import unittest
from types import SimpleNamespace

from opendbc.car.psa.tests.test_longitudinal import LongitudinalHarness


class FakeLeadSubMaster:
  def __init__(self, *, present=True, d_rel=31.0, v_rel=-0.1, model_speed=22.04):
    self.seen = {'radarState': True, 'modelV2': True}
    self.valid = {'radarState': True, 'modelV2': True}
    self.alive = {'radarState': True, 'modelV2': True}
    self._data = {
      'radarState': SimpleNamespace(
        leadOne=SimpleNamespace(present=present, dRel=d_rel, vRel=v_rel),
      ),
      'modelV2': SimpleNamespace(
        leadsV3=[SimpleNamespace(x=[d_rel], v=[model_speed])] if present else [],
      ),
    }

  def update(self, timeout=0):
    return None

  def __getitem__(self, key):
    return self._data[key]


class TestArtivLeadEmulation(unittest.TestCase):
  def setUp(self):
    self.h = LongitudinalHarness()
    self.h.activate()
    self.h.cs.out.vEgo = 22.14
    self.h.cs.out.vEgoRaw = 22.14
    self.h.cc.hudControl.leadVisible = True
    self.h.controller.model_sm = FakeLeadSubMaster()

  def next_radar_bundle(self):
    for _ in range(20):
      _, messages = self.h.step()
      by_address = {a: (d, self.h.decode(a, d)) for a, d, _ in messages if a in (0x2F6, 0x4F6)}
      if 0x2F6 in by_address and 0x4F6 in by_address:
        return by_address
    self.fail('did not observe a simultaneous 0x2F6/0x4F6 emission')

  def test_one_lead_populates_both_2f6_and_4f6(self):
    values = self.next_radar_bundle()
    f6 = values[0x2F6][1]
    f46 = values[0x4F6][1]

    self.assertEqual(f6['TARGET_DETECTED'], 1)
    self.assertEqual(f6['INTER_VEHICLE_DISTANCE'], 31.0)
    self.assertEqual(f6['DISPLAY_INTERVEHICLE_TIME'], 1.4)
    self.assertEqual(f6['TARGET_POSITION'], 1)

    self.assertEqual(f46['TARGET_DETECTED'], 1)
    self.assertEqual(f46['DISTANCE_GAP'], 31.0)
    self.assertEqual(f46['TIME_GAP'], 1.4)
    self.assertAlmostEqual(f46['RELATIVE_SPEED'], -0.1, places=2)
    self.assertEqual(f46['ARTIV_SENSOR_STATE'], 2)
    self.assertEqual(f46['ARTIV_TARGET_CHANGE_INFO'], 0)

  def test_no_lead_keeps_recorded_no_target_sentinels(self):
    self.h.cc.hudControl.leadVisible = False
    self.h.controller.model_sm = FakeLeadSubMaster(present=False)
    values = self.next_radar_bundle()
    f6 = values[0x2F6][1]
    f46_raw, f46 = values[0x4F6]

    self.assertEqual(f6['TARGET_DETECTED'], 0)
    self.assertEqual(f6['INTER_VEHICLE_DISTANCE'], 255.5)
    self.assertEqual(f6['DISPLAY_INTERVEHICLE_TIME'], 6.2)

    self.assertEqual(f46['TARGET_DETECTED'], 0)
    self.assertEqual(f46['TIME_GAP'], 25.5)
    self.assertEqual(f46['DISTANCE_GAP'], 254.0)
    self.assertAlmostEqual(f46['RELATIVE_SPEED'], 93.8, places=2)
    self.assertEqual(f46_raw.hex(), 'fffe5ffe00')


if __name__ == '__main__':
  unittest.main()
