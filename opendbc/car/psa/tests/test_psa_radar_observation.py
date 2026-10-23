"""Passive stock ARTIV telemetry: no CAN transmission or synthetic radar tracks."""
from types import SimpleNamespace

from opendbc.car import Bus, structs
from opendbc.car.psa.carstate import CarState
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.radar_observation import decode_stock_artiv_lead
from opendbc.car.psa.values import CAR


NS = 1_000_000_000
DISTANCE = {"TARGET_DETECTED": 1, "INTER_VEHICLE_DISTANCE": 27.25}
MOTION = {"TARGET_DETECTED": 1, "ARTIV_SENSOR_STATE": 2,
          "DISTANCE_GAP": 27.0, "RELATIVE_SPEED": -3.5}


def test_matching_stock_lead_is_recorded_in_standard_carstate_fields():
  result = decode_stock_artiv_lead(DISTANCE, NS, MOTION, NS + 10_000_000, NS + 20_000_000)
  assert result.distance_valid and result.relative_speed_valid
  assert result.distance_m == 27.25
  assert result.relative_speed_mps == -3.5
  assert result.measurement_time_nanos == NS

  cs = structs.CarState()
  cs.stockRadarLeadDistance = result.distance_m
  cs.stockRadarLeadRelativeSpeed = result.relative_speed_mps
  cs.stockRadarLeadValid = result.distance_valid
  cs.stockRadarRelativeSpeedValid = result.relative_speed_valid
  cs.stockRadarMeasurementTimeNanos = result.measurement_time_nanos
  assert cs.stockRadarLeadValid and cs.stockRadarRelativeSpeedValid


def test_absent_or_stale_distance_never_reports_a_lead():
  assert not decode_stock_artiv_lead(DISTANCE, 0, MOTION, NS, NS).distance_valid
  assert not decode_stock_artiv_lead(DISTANCE, NS, MOTION, NS, NS + 400_000_000).distance_valid
  assert not decode_stock_artiv_lead({**DISTANCE, "TARGET_DETECTED": 0}, NS, MOTION, NS, NS).distance_valid
  assert not decode_stock_artiv_lead({**DISTANCE, "INTER_VEHICLE_DISTANCE": 255.5}, NS, MOTION, NS, NS).distance_valid


def test_missing_or_inconsistent_motion_does_not_invent_relative_speed():
  samples = [
    (MOTION, NS + 200_000_000),
    ({**MOTION, "TARGET_DETECTED": 0}, NS),
    ({**MOTION, "DISTANCE_GAP": 70.0}, NS),
    ({**MOTION, "ARTIV_SENSOR_STATE": 4}, NS),
    ({**MOTION, "DISTANCE_GAP": 254.0}, NS),
  ]
  for motion, timestamp in samples:
    result = decode_stock_artiv_lead(DISTANCE, NS, motion, timestamp, NS + 200_000_000)
    assert result.distance_valid
    assert not result.relative_speed_valid
    assert result.relative_speed_mps == 0.0


def test_adas_parser_monitors_optional_2f6_and_4f6_without_can_valid_requirement():
  cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_3008)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.PSA_PEUGEOT_3008)
  parser = CarState.get_can_parsers(cp, cp_sp)[Bus.adas]
  assert 0x2F6 in parser.addresses and 0x4F6 in parser.addresses
  assert parser.message_states[0x2F6].ignore_alive
  assert parser.message_states[0x4F6].ignore_alive


def test_dashcam_keeps_stock_artiv_enabled():
  from opendbc.car.psa.carcontroller import CarController

  cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_3008)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.PSA_PEUGEOT_3008)
  assert cp.dashcamOnly
  controller = CarController({Bus.main: "psa_aee2010_r3"}, cp, cp_sp)
  assert not controller.longitudinal_enabled
  cs = SimpleNamespace(eps_active=False, out=structs.CarState())
  cs.out.canValid = True
  for frame in (0, 100, 1_000, 10_000):
    controller.frame = frame
    _, can_sends = controller.update(structs.CarControl().as_reader(), structs.CarControlSP(), cs, 0)
    assert not any(msg.address == 0x6B6 for msg in can_sends)
