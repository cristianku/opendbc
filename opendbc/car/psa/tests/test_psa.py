import pytest

from opendbc.can.dbc import DBC
from opendbc.can.packer import CANPacker
from opendbc.can.parser import get_raw_value
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.psacan import psa_checksum, set_speed
from opendbc.car.psa.values import CAR


def test_icbm_available_with_stock_longitudinal():
  cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_3008)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.PSA_PEUGEOT_3008)

  assert not cp.openpilotLongitudinalControl
  assert cp_sp.intelligentCruiseButtonManagementAvailable
  assert not cp_sp.pcmCruiseSpeed


@pytest.mark.parametrize(("setpoint", "parity"), (
  (57, 0),
  (61, 1),
  (64, 2),
  (67, 2),
  (72, 3),
  (74, 2),
  (75, 3),
  (117, 2),
  (255, 0),
))
def test_set_speed_parity_and_checksum(setpoint, parity):
  packer = CANPacker("psa_aee2010_r3")
  dbc = DBC("psa_aee2010_r3")
  msg_def = dbc.addr_to_msg[0x452]
  values = {"COUNTER": 7, "SPEED_SETPOINT": 42}

  address, dat, _ = set_speed(packer, values, setpoint)

  assert values == {"COUNTER": 7, "SPEED_SETPOINT": 42}
  assert get_raw_value(dat, msg_def.sigs["SPEED_SETPOINT"]) == setpoint
  assert get_raw_value(dat, msg_def.sigs["CHECKSUM_CONS_RVV_LVV2"]) == parity

  checksum_sig = msg_def.sigs["CHECKSUM"]
  assert get_raw_value(dat, checksum_sig) == psa_checksum(address, checksum_sig, bytearray(dat))
