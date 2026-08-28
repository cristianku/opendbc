from types import SimpleNamespace

from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.psa import carcontroller as psa_carcontroller
from opendbc.car.psa.carcontroller import get_eps_takeover_delay_frames, should_preempt_eps_rearm
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


def test_c4_eps_takeover_straight_delay_matches_observed_reactivation():
  cp = CarInterface.get_non_essential_params(CAR.PSA_CITROEN_C4_SPACETOURER)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.PSA_CITROEN_C4_SPACETOURER)
  controller = psa_carcontroller.CarController({Bus.main: 'psa_aee2010_r3'}, cp, cp_sp)
  controller.activation_request_frame = 100

  cs = SimpleNamespace(eps_active=False, out=structs.CarState())
  cs.out.vEgo = 20.0

  controller.frame = 419
  controller._activate_eps(cs, 0.0)
  assert controller.takeover_req == 0

  controller.frame = 420
  controller._activate_eps(cs, 0.0)
  assert controller.takeover_req == 1


def test_c4_eps_takeover_full_curve_delay_remains_urgent():
  cp = CarInterface.get_non_essential_params(CAR.PSA_CITROEN_C4_SPACETOURER)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.PSA_CITROEN_C4_SPACETOURER)
  controller = psa_carcontroller.CarController({Bus.main: 'psa_aee2010_r3'}, cp, cp_sp)
  controller.activation_request_frame = 100

  cs = SimpleNamespace(eps_active=False, out=structs.CarState())
  cs.out.vEgo = 20.0

  controller.frame = 149
  controller._activate_eps(cs, 0.005)
  assert controller.takeover_req == 0

  controller.frame = 150
  controller._activate_eps(cs, 0.005)
  assert controller.takeover_req == 1


# [eps curve] - START
def test_eps_rearm_is_preempted_for_curve_inside_fixed_lookahead():
  model_t = [0.0, 3.0, 5.0, 5.1, 6.0]
  model_yaw_rate = [0.0, 0.0, 0.04, 0.0, 0.0]
  model_speed = [20.0] * len(model_t)

  assert CarControllerParams.EPS_REARM_CURVE_LOOKAHEAD == 5.0
  assert should_preempt_eps_rearm(3.0, 20.0, 0.0, model_t, model_yaw_rate, model_speed)


def test_eps_rearm_is_not_preempted_too_early_or_beyond_lookahead():
  model_t = [0.0, 3.0, 5.0, 5.1, 6.0]
  model_yaw_rate = [0.0, 0.0, 0.0, 0.04, 0.04]
  model_speed = [20.0] * len(model_t)

  assert not should_preempt_eps_rearm(2.99, 20.0, 0.0, model_t, model_yaw_rate, model_speed)
  assert not should_preempt_eps_rearm(3.0, 20.0, 0.0, model_t, model_yaw_rate, model_speed)


def test_eps_rearm_is_not_preempted_while_already_cornering():
  model_t = [0.0, 3.0, 4.0, 5.0]
  model_yaw_rate = [0.04] * len(model_t)
  model_speed = [20.0] * len(model_t)

  assert not should_preempt_eps_rearm(4.0, 20.0, 0.002, model_t, model_yaw_rate, model_speed)
# [eps curve] - END


def test_eps_takeover_is_requested_when_curve_reaches_rearm_deadline():
  model_t = [0.0, 0.5, 1.0, 1.5, 2.0]
  model_yaw_rate = [0.04] * len(model_t)
  model_speed = [20.0] * len(model_t)

  assert psa_carcontroller.should_request_eps_takeover(
    10.0, 20.0, 0.002, False, True, model_t, model_yaw_rate, model_speed,
  )


def test_eps_takeover_is_not_requested_before_warning_window():
  assert not psa_carcontroller.should_request_eps_takeover(
    9.99, 20.0, 0.002, False, False, [], [], [],
  )


def test_eps_takeover_is_not_requested_while_currently_straight():
  assert not psa_carcontroller.should_request_eps_takeover(
    10.0, 20.0, 0.0005, False, False, [], [], [],
  )


def test_eps_takeover_is_not_repeated_after_shared_latch_is_set():
  assert not psa_carcontroller.should_request_eps_takeover(
    10.0, 20.0, 0.002, True, False, [], [], [],
  )


def test_eps_takeover_is_suppressed_when_curve_ends_before_rearm():
  model_t = [0.0, 0.5, 1.0, 1.5, 2.0]
  model_yaw_rate = [0.04, 0.04, 0.0, 0.0, 0.0]
  model_speed = [20.0] * len(model_t)

  assert not psa_carcontroller.should_request_eps_takeover(
    10.0, 20.0, 0.002, False, True, model_t, model_yaw_rate, model_speed,
  )


def test_eps_takeover_is_requested_without_stable_straight_before_rearm():
  model_t = [0.0, 0.5, 1.0, 1.5, 2.0]
  model_yaw_rate = [0.04, 0.0, 0.0, 0.04, 0.0]
  model_speed = [20.0] * len(model_t)

  assert psa_carcontroller.should_request_eps_takeover(
    10.0, 20.0, 0.002, False, True, model_t, model_yaw_rate, model_speed,
  )


def test_eps_takeover_is_requested_when_model_is_invalid():
  assert psa_carcontroller.should_request_eps_takeover(
    10.0, 20.0, 0.002, False, False, [], [], [],
  )


def test_eps_takeover_is_requested_when_prediction_does_not_reach_rearm():
  assert psa_carcontroller.should_request_eps_takeover(
    10.0, 20.0, 0.002, False, True, [0.0, 0.5, 1.0], [0.04, 0.0, 0.0], [20.0, 20.0, 20.0],
  )


def test_eps_takeover_is_requested_when_prediction_contains_invalid_data():
  assert psa_carcontroller.should_request_eps_takeover(
    10.0, 20.0, 0.002, False, True,
    [0.0, 0.5, 1.0, 1.5, 2.0], [0.04, 0.0, float('nan'), 0.0, 0.0], [20.0] * 5,
  )


def test_eps_takeover_is_requested_when_prediction_timestamps_are_unordered():
  assert psa_carcontroller.should_request_eps_takeover(
    10.0, 20.0, 0.002, False, True,
    [0.0, 1.9, 100.0, 2.0], [0.04, 0.0, 0.0, 0.04], [20.0] * 4,
  )


def test_eps_takeover_is_requested_when_deadline_bracket_is_too_sparse():
  assert psa_carcontroller.should_request_eps_takeover(
    10.0, 20.0, 0.002, False, True,
    [0.0, 1.9, 100.0], [0.04, 0.0, 0.0], [20.0] * 3,
  )


def test_eps_takeover_is_requested_when_prediction_vectors_are_misaligned():
  assert psa_carcontroller.should_request_eps_takeover(
    10.0, 20.0, 0.002, False, True,
    [0.0, 1.0, 1.5, 2.0], [0.04, 0.0, 0.0, 0.0, 0.0], [20.0] * 4,
  )


def test_eps_deactivation_keeps_shared_takeover_latch_set():
  controller = psa_carcontroller.CarController.__new__(psa_carcontroller.CarController)
  controller.frame = 1200
  controller.takeover_req_already_sent = True

  controller._deactivate_eps()

  assert controller.takeover_req_already_sent


def test_lateral_reset_clears_shared_takeover_latch():
  controller = psa_carcontroller.CarController.__new__(psa_carcontroller.CarController)
  controller.takeover_req_already_sent = True

  controller._reset_lat_state()

  assert not controller.takeover_req_already_sent


def test_eps_takeover_request_sets_existing_shared_latch():
  cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_3008)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.PSA_PEUGEOT_3008)
  controller = psa_carcontroller.CarController({Bus.main: 'psa_aee2010_r3'}, cp, cp_sp)
  controller.frame = 1200
  controller.eps_activation_frame = 200
  controller.model_sm = None

  controller._maybe_request_eps_takeover(20.0, 0.002)

  assert controller.takeover_req == 1
  assert controller.takeover_req_already_sent


def test_new_eps_active_cycle_clears_shared_takeover_latch():
  controller = psa_carcontroller.CarController.__new__(psa_carcontroller.CarController)
  controller.frame = 1300
  controller.eps_activation_frame = 0
  controller.takeover_req_already_sent = True

  controller._start_eps_active_cycle()

  assert controller.eps_activation_frame == 1300
  assert not controller.takeover_req_already_sent


def test_active_eps_sends_takeover_without_releasing_steering_torque():
  cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_3008)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.PSA_PEUGEOT_3008)
  controller = psa_carcontroller.CarController({Bus.main: 'psa_aee2010_r3'}, cp, cp_sp)
  controller.model_sm = None
  controller.frame = 1005
  controller.eps_activation_frame = 5

  cc = structs.CarControl()
  cc.latActive = True
  cc.actuators.torque = 0.5
  cc.actuators.curvature = 0.002
  cs = SimpleNamespace(
    eps_active=True,
    speed_kph=72.0,
    out=structs.CarState(),
    HS2_DYN_MDD_ETAT_2F6={},
  )
  cs.out.vEgo = 20.0
  cs.out.steeringPressed = False
  cs.out.steeringTorque = 0.0

  actuators, _ = controller.update(cc.as_reader(), structs.CarControlSP(), cs, 0)
  actuators_next, can_sends = controller.update(cc.as_reader(), structs.CarControlSP(), cs, 0)

  assert controller.takeover_req_already_sent
  assert controller.status == 4
  assert actuators.torque > 0.0
  assert actuators_next.torque == actuators.torque

  takeover_msgs = [msg for msg in can_sends if msg[0] == 0x2F6]
  assert len(takeover_msgs) == 1
  assert (takeover_msgs[0][1][0] >> 1) & 0x3 == 1


def test_c4_spacetourer_uses_20_second_eps_rearm_deadline():
  cp = CarInterface.get_non_essential_params(CAR.PSA_CITROEN_C4_SPACETOURER)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.PSA_CITROEN_C4_SPACETOURER)
  controller = psa_carcontroller.CarController({Bus.main: 'psa_aee2010_r3'}, cp, cp_sp)
  controller.model_sm = None
  controller.frame = 1205
  controller.eps_activation_frame = 5

  cc = structs.CarControl()
  cc.latActive = True
  cs = SimpleNamespace(eps_active=True, speed_kph=72.0, out=structs.CarState())
  cs.out.vEgo = 20.0
  cs.out.steeringPressed = False
  cs.out.steeringTorque = 0.0

  controller.update(cc.as_reader(), structs.CarControlSP(), cs, 0)
  assert controller.status == 4

  controller.frame = 2005
  controller.update(cc.as_reader(), structs.CarControlSP(), cs, 0)
  assert controller.status == 2
  assert controller.deactivation_in_progress


def test_peugeot_3008_keeps_12_second_eps_rearm_deadline():
  cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_3008)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.PSA_PEUGEOT_3008)
  controller = psa_carcontroller.CarController({Bus.main: 'psa_aee2010_r3'}, cp, cp_sp)
  controller.model_sm = None
  controller.frame = 1200
  controller.eps_activation_frame = 5

  cc = structs.CarControl()
  cc.latActive = True
  cs = SimpleNamespace(eps_active=True, speed_kph=72.0, out=structs.CarState())
  cs.out.vEgo = 20.0
  cs.out.steeringPressed = False
  cs.out.steeringTorque = 0.0

  controller.update(cc.as_reader(), structs.CarControlSP(), cs, 0)
  assert controller.status == 4

  controller.frame = 1205
  controller.update(cc.as_reader(), structs.CarControlSP(), cs, 0)
  assert controller.status == 2
  assert controller.deactivation_in_progress


def test_c4_spacetourer_takeover_warning_moves_to_18_seconds():
  cp = CarInterface.get_non_essential_params(CAR.PSA_CITROEN_C4_SPACETOURER)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.PSA_CITROEN_C4_SPACETOURER)
  controller = psa_carcontroller.CarController({Bus.main: 'psa_aee2010_r3'}, cp, cp_sp)
  controller.model_sm = None
  controller.eps_activation_frame = 5

  controller.frame = 1005
  controller._maybe_request_eps_takeover(20.0, 0.002)
  assert controller.takeover_req == 0
  assert not controller.takeover_req_already_sent

  controller.frame = 1805
  controller._maybe_request_eps_takeover(20.0, 0.002)
  assert controller.takeover_req == 1
  assert controller.takeover_req_already_sent
