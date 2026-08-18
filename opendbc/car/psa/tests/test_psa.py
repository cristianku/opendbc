from opendbc.car import DT_CTRL, structs
from opendbc.car.psa.carcontroller import get_eps_takeover_delay_frames
from opendbc.car.psa.carstate import CarState
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.psacan import create_disable_radar
from opendbc.car.psa.values import CAR, CarControllerParams, PSA_ADAS_BUS


def test_icbm_unavailable_with_stock_longitudinal():
  cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_3008)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.PSA_PEUGEOT_3008)

  assert not cp.openpilotLongitudinalControl
  assert not cp_sp.intelligentCruiseButtonManagementAvailable
  assert cp_sp.pcmCruiseSpeed


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


def test_eps_takeover_delay_is_bounded_by_lateral_acceleration():
  max_frames = round(CarControllerParams.EPS_ACTIVATE_TAKEOVER_MAX_PERIOD / DT_CTRL)
  min_frames = round(CarControllerParams.EPS_ACTIVATE_TAKEOVER_MIN_PERIOD / DT_CTRL)

  assert get_eps_takeover_delay_frames(20.0, 0.0) == max_frames
  assert get_eps_takeover_delay_frames(10.0, 0.005) == 45  # 0.5 m/s^2: halfway between the bounds
  assert get_eps_takeover_delay_frames(20.0, 0.005) == min_frames
  assert get_eps_takeover_delay_frames(40.0, 0.1) == min_frames


def test_eps_takeover_delay_matches_observed_rearm():
  # Last observed false-positive rearm: 56.7 km/h at curvature 0.002735.
  # The bounded timeout is ~360 ms instead of the previous immediate request.
  assert get_eps_takeover_delay_frames(56.7 / 3.6, 0.002735) == 36
