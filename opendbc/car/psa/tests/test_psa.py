import pytest

from opendbc.can.dbc import DBC
from opendbc.can.packer import CANPacker
from opendbc.can.parser import get_raw_value
from opendbc.car import structs
from opendbc.car.psa.carstate import CarState
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.psacan import create_disable_radar, psa_checksum, set_speed
from opendbc.car.psa.values import CAR, PSA_ADAS_BUS


def test_icbm_available_with_stock_longitudinal():
  cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_3008)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.PSA_PEUGEOT_3008)

  assert not cp.openpilotLongitudinalControl
  assert cp_sp.intelligentCruiseButtonManagementAvailable
  assert not cp_sp.pcmCruiseSpeed


def test_synthetic_cruise_button_events_follow_stock_setpoint():
  cs = CarState.__new__(CarState)
  cs.synthetic_cruise_kph = None
  cs.synthetic_cruise_button_pressed = False
  cs.synthetic_cruise_button_type = structs.CarState.ButtonEvent.Type.unknown
  cs.cruise_enabled_prev = False

  assert cs._update_cruise_button_events(60, True) == []

  events = cs._update_cruise_button_events(62, True)
  assert [(event.type, event.pressed) for event in events] == [
    (structs.CarState.ButtonEvent.Type.accelCruise, True),
  ]
  events = cs._update_cruise_button_events(62, True)
  assert [(event.type, event.pressed) for event in events] == [
    (structs.CarState.ButtonEvent.Type.accelCruise, False),
  ]
  assert cs.synthetic_cruise_kph == 61

  events = cs._update_cruise_button_events(60, True)
  assert [(event.type, event.pressed) for event in events] == [
    (structs.CarState.ButtonEvent.Type.decelCruise, True),
  ]
  events = cs._update_cruise_button_events(60, True)
  assert [(event.type, event.pressed) for event in events] == [
    (structs.CarState.ButtonEvent.Type.decelCruise, False),
  ]
  assert cs.synthetic_cruise_kph == 60

  assert cs._update_cruise_button_events(255, False) == []
  assert cs.synthetic_cruise_kph is None


def test_disable_radar_programming_session():
  msg = create_disable_radar()

  assert msg.address == 0x6B6
  assert msg.src == PSA_ADAS_BUS
  assert msg.dat == b"\x02\x10\x02\x00\x00\x00\x00\x00"


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
