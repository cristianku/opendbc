# [psa fingerprint] - START
import os
import unittest
from unittest.mock import patch

from opendbc.car.car_helpers import fingerprint
from opendbc.car.psa.values import CAR
from opendbc.car.structs import CarParams
from opendbc.car.vin import VIN_UNKNOWN


class TestPeugeotFingerprintMultiplexing(unittest.TestCase):
  VIN = 'VF3M45GFRKS000001'  # Synthetic VIN used only to simulate an ECU response.
  VERSIONS = {0x6A8: b'000D170047100', 0x6A9: b'1614191B101502000000', 0x6AD: b'085065201906190129'}

  def setUp(self):
    self.modes = []
    self.queries = []
    self.vin_available = True
    self.enterContext(patch.dict(os.environ, {'FINGERPRINT': '', 'SKIP_FW_QUERY': '', 'DISABLE_FW_CACHE': '', 'REPLAY': ''}))

    # Replace the vehicle transport, keeping VIN parsing, query selection, and FW matching real.
    def get_data(query, timeout):
      self.queries.append((query.bus, query.request))
      if query.bus != 0:
        return {}
      if query.request == [b'\x22\xf1\x90'] and self.vin_available:
        return {(0x7E0, None): self.VIN.encode()}
      if query.request == [b'\x10\x01', b'\x22\xf1\x8c']:
        return {(addr, None): version for addr, version in self.VERSIONS.items() if (addr, None) in query.msg_addrs}
      return {}

    self.enterContext(patch('opendbc.car.isotp_parallel_query.IsoTpParallelQuery.get_data', get_data))
    self.enterContext(patch('opendbc.car.fw_versions.get_ecu_addrs', return_value={(0x688, None, 0), (0x689, None, 0), (0x68D, None, 0)}))
    self.enterContext(patch('opendbc.car.car_helpers.time.sleep'))

  def run_fingerprint(self, platform=CAR.PSA_PEUGEOT_3008, cached_params=None):
    return fingerprint(lambda **kwargs: [[]], lambda msgs: None, self.modes.append, cached_params, platform)

  def assert_peugeot_without_obd_switch(self, result):
    candidate, _, vin, firmware, source, exact = result
    self.assertEqual(candidate, CAR.PSA_PEUGEOT_3008)
    self.assertEqual(vin, self.VIN)
    self.assertEqual(source, CarParams.FingerprintSource.fixed)
    self.assertTrue(exact)
    self.assertEqual({(fw.address, bytes(fw.fwVersion)) for fw in firmware}, set(self.VERSIONS.items()))
    self.assertTrue(all(fw.brand == 'psa' and not fw.obdMultiplexing for fw in firmware))
    self.assertTrue(self.modes)
    self.assertNotIn(True, self.modes)

  def test_selected_peugeot_reads_vin_and_firmware_without_switching_radar_bus(self):
    for vin in (self.VIN, 'VR3M45GFRMS000123'):
      with self.subTest(vin=vin):
        self.VIN = vin
        self.modes.clear()
        self.assert_peugeot_without_obd_switch(self.run_fingerprint())

  def test_environment_selection_uses_same_peugeot_path(self):
    with patch.dict(os.environ, {'FINGERPRINT': str(CAR.PSA_PEUGEOT_3008)}):
      self.assert_peugeot_without_obd_switch(self.run_fingerprint(platform=None))

  def test_missing_vin_does_not_fall_back_to_radar_bus_or_enable_multiplexing(self):
    self.vin_available = False
    candidate, _, vin, firmware, source, _ = self.run_fingerprint()
    self.assertEqual(candidate, CAR.PSA_PEUGEOT_3008)
    self.assertEqual(vin, VIN_UNKNOWN)
    self.assertEqual(source, CarParams.FingerprintSource.fixed)
    self.assertEqual({fw.address for fw in firmware}, {0x6A8, 0x6A9, 0x6AD})
    self.assertNotIn(True, self.modes)
    vin_query_buses = {bus for bus, requests in self.queries if len(requests) == 1}
    self.assertEqual(vin_query_buses, {0})

  def test_automatic_and_other_platform_selection_keep_existing_multiplexing(self):
    for platform in (None, CAR.PSA_PEUGEOT_208, CAR.PSA_CITROEN_C4_SPACETOURER):
      with self.subTest(platform=platform):
        self.modes.clear()
        candidate, _, vin, firmware, source, _ = self.run_fingerprint(platform)
        self.assertEqual(candidate, platform or CAR.PSA_PEUGEOT_3008)
        self.assertEqual(vin, self.VIN)
        self.assertTrue(firmware)
        self.assertIn(True, self.modes)
        self.assertFalse(self.modes[-1])
        self.assertEqual(source, CarParams.FingerprintSource.fixed if platform else CarParams.FingerprintSource.fw)

  def test_valid_cache_preserves_vin_and_firmware_without_new_queries(self):
    cp = CarParams(brand='psa', carFingerprint=str(CAR.PSA_PEUGEOT_3008), carVin=self.VIN,
                   carFw=[CarParams.CarFw(ecu=CarParams.Ecu.engine, brand='psa', address=0x6A8,
                                         fwVersion=self.VERSIONS[0x6A8])])
    candidate, _, vin, firmware, source, _ = self.run_fingerprint(cached_params=cp)
    self.assertEqual(candidate, CAR.PSA_PEUGEOT_3008)
    self.assertEqual(vin, self.VIN)
    self.assertEqual([fw.to_dict() for fw in firmware], [fw.to_dict() for fw in cp.carFw])
    self.assertEqual(source, CarParams.FingerprintSource.fixed)
    self.assertEqual(self.queries, [])
    self.assertNotIn(True, self.modes)

    with patch.dict(os.environ, {'DISABLE_FW_CACHE': '1'}):
      self.assert_peugeot_without_obd_switch(self.run_fingerprint(cached_params=cp))

  def test_skip_fw_query_still_skips_diagnostics(self):
    with patch.dict(os.environ, {'SKIP_FW_QUERY': '1'}):
      candidate, _, vin, firmware, source, _ = self.run_fingerprint()
    self.assertEqual(candidate, CAR.PSA_PEUGEOT_3008)
    self.assertEqual(vin, VIN_UNKNOWN)
    self.assertEqual(firmware, [])
    self.assertEqual(source, CarParams.FingerprintSource.fixed)
    self.assertEqual(self.queries, [])
    self.assertNotIn(True, self.modes)
# [psa fingerprint] - END
