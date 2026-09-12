from opendbc.can.packer import CANPacker
# [CLAUDE eps-rearm] - START
from opendbc.car import Bus, structs, DT_CTRL
# [CLAUDE eps-rearm] - END
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.can_definitions import CanData
from opendbc.car.psa.psacan import (
  # create_driver_torque,
  create_lka_steering,
  create_request_takeover,
  # create_resume_acc,
  create_steering_hold,
  create_disable_radar,
  create_HS2_DYN1_MDD_ETAT_2B6,
  create_HS2_DYN_MDD_ETAT_2F6,
  create_HS2_DAT_ARTIV_V2_4F6,
  create_HS2_SUPV_ARTIV_796,
)
from opendbc.car.psa.values import CarControllerParams, CAR, LKAS_LIMITS, PSA_ADAS_BUS
from opendbc.car.psa.neutral_radar import NeutralRadar
# [psa longitudinal] - START
from numpy import interp
from opendbc.car.psa.values import LongitudinalParams, PSA_LONG_CONTROL
# [psa longitudinal] - END

try:
  import openpilot.cereal.messaging as messaging
except ImportError:
  try:
    # Compatibility with older openpilot trees, where cereal was top-level.
    from cereal import messaging
  except ImportError:
    # Standalone opendbc tests do not ship cereal.
    messaging = None

import random
import math

SteerControlType = structs.CarParams.SteerControlType


# [artiv probe] - START
ARTIV_PROGRAMMING_WAIT = 10.0  # seconds of valid CAN at standstill before the one-shot request
# [artiv probe] - END


# [eps curve] - START
def should_preempt_eps_rearm(elapsed, v_ego, current_curvature, model_t, model_yaw_rate, model_speed):
  """Return True when an upcoming curve makes the current straight a good rearm opportunity."""
  if elapsed < CarControllerParams.EPS_REARM_EARLIEST_PERIOD:
    return False

  current_lat_accel = abs(current_curvature) * v_ego ** 2
  if current_lat_accel > CarControllerParams.EPS_REARM_STRAIGHT_LAT_ACCEL:
    return False

  for t, yaw_rate, speed in zip(model_t, model_yaw_rate, model_speed, strict=False):
    if not (math.isfinite(t) and math.isfinite(yaw_rate) and math.isfinite(speed)):
      continue
    if 0.0 < t <= CarControllerParams.EPS_REARM_CURVE_LOOKAHEAD:
      # yaw rate [rad/s] * forward speed [m/s] = lateral acceleration [m/s^2]
      if abs(yaw_rate * speed) >= CarControllerParams.EPS_REARM_CURVE_LAT_ACCEL:
        return True

  return False
# [eps curve] - END


def should_request_eps_takeover(elapsed, v_ego, current_curvature, takeover_req_already_sent,
                                model_valid, model_t, model_yaw_rate, model_speed,
                                eps_rearm_period=CarControllerParams.EPS_REARM_PERIOD):
  """Warn before the fixed EPS rearm unless a stable straight is predicted by the deadline."""
  if takeover_req_already_sent:
    return False

  remaining = eps_rearm_period - elapsed
  if remaining <= 0.0 or remaining > CarControllerParams.EPS_TAKEOVER_WARNING_PERIOD:
    return False

  current_lat_accel = abs(current_curvature) * v_ego ** 2
  if current_lat_accel < CarControllerParams.EPS_REARM_CURVE_LAT_ACCEL:
    return False

  if not model_valid:
    return True

  if len(model_t) != len(model_yaw_rate) or len(model_t) != len(model_speed) or len(model_t) < 2:
    return True

  last_t = None
  for t, yaw_rate, speed in zip(model_t, model_yaw_rate, model_speed, strict=True):
    if not (math.isfinite(t) and math.isfinite(yaw_rate) and math.isfinite(speed)):
      return True
    if t < 0.0 or (last_t is not None and t <= last_t):
      return True
    last_t = t

  previous_t = None
  previous_straight = False
  straight_at_deadline = False
  deadline_covered = False
  for t, yaw_rate, speed in zip(model_t, model_yaw_rate, model_speed, strict=True):
    if t <= 0.0:
      continue

    predicted_lat_accel = abs(yaw_rate * speed)
    predicted_straight = predicted_lat_accel <= CarControllerParams.EPS_REARM_STRAIGHT_LAT_ACCEL
    if t >= remaining:
      deadline_covered = True
      deadline_bracket_valid = previous_t is not None and t - previous_t <= CarControllerParams.EPS_TAKEOVER_MODEL_MAX_TIME_GAP
      straight_at_deadline = deadline_bracket_valid and previous_straight and predicted_straight
      break
    previous_t = t
    previous_straight = predicted_straight

  return not (deadline_covered and straight_at_deadline)

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    self.latActiveLast = False
    self.eps_active_last = False
    self.packer = CANPacker(dbc_names[Bus.main])
    self.apply_torque_scaled_last = 0
    self.apply_can_torque_last = 0  # raw CAN torque logged to steeringAngleDeg (debug); init so it always exists
    self.apply_torque_factor = 0
    self.apply_torque = 0
    self.status = 2
    self.takeover_req = 0
    self.start_takeover_repeats = 0
    # Shared latch for both the pre-rearm warning and an immediate curve warning during EPS reactivation.
    self.takeover_req_already_sent = False
    self.model_sm = messaging.SubMaster(['modelV2']) if messaging is not None else None

    # this is the frame when the latactive is being pressed
    self.car_fingerprint = CP.carFingerprint
    self.params = CarControllerParams(CP)
    self.radar_disabled = False
    # [psa longitudinal] - START
    self.longitudinal_profile = self.car_fingerprint == CAR.PSA_PEUGEOT_3008 and CP.openpilotLongitudinalControl
    self.longitudinal_enabled = (self.longitudinal_profile and not CP.dashcamOnly and not CP.passive
                                 and any(c.safetyModel == structs.CarParams.SafetyModel.psa and c.safetyParam & PSA_LONG_CONTROL
                                         for c in CP.safetyConfigs))
    # [psa longitudinal] - END
    # [artiv probe] - START
    self.artiv_programming_requested = False
    self.artiv_probe_last_frame = 0
    # [neutral motion] - START
    # Keep substitutes and TesterPresent running after a parked start, including in reverse.
    # Actuation remains gated separately by longitudinal_enabled in _update_longitudinal.
    self.neutral_radar = NeutralRadar(stationary_only=False) if self.car_fingerprint == CAR.PSA_PEUGEOT_3008 else None
    # [neutral motion] - END
    # [artiv probe] - END
    self.bars = 4
    self.steering_hold_counter = 0
    self.next_steering_hold = random.randint(8, 12)  # ~10Hz con jitter ±20%
    self.driver_torque_counter = 0
    self.next_driver_torque = random.randint(500, 800)  # 5–8 s @100 Hz
    self.last_activation_frame = 0
    self.eps_activation_frame = 0
    self.creep_start_frame = 0
    self.last_status_change_frame = 0     # frame dell'ultimo cambio di gradino
    self.deactivation_in_progress = False
    self.eps_rearm_frames = int(self.params.EPS_REARM_PERIOD / DT_CTRL)
    self.eps_state_last = 0
    self.takeover_msg_duration = int(self.params.TAKEOVER_MSG_DURATION / DT_CTRL)   # 0.1 s = 10 frame

  # [psa longitudinal] - START
  def _update_longitudinal(self, CC, CS):
    """Prepare explicit CAN inputs. Experimental torque mapping; no emission or scheduling here."""
    self.longitudinal_active = False
    self.longitudinal_braking = False
    self.longitudinal_accel = 0.0
    self.longitudinal_potential_torque = LongitudinalParams.INACTIVE_TORQUE
    self.longitudinal_wheel_torque = LongitudinalParams.INACTIVE_TORQUE
    self.longitudinal_min_time = 0.0
    if not (self.longitudinal_enabled and self.neutral_radar.active and CC.enabled and CC.longActive
            and CS.out.canValid and not CS.out.gasPressed and not CS.out.brakePressed
            and math.isfinite(CC.actuators.accel)):
      return

    self.longitudinal_active = True
    self.longitudinal_accel = max(LongitudinalParams.ACCEL_LOOKUP[0], min(CC.actuators.accel, LongitudinalParams.ACCEL_LOOKUP[-1]))
    self.longitudinal_braking = self.longitudinal_accel < LongitudinalParams.BRAKE_ACCEL_THRESHOLD
    if not self.longitudinal_braking:
      torque = float(interp(self.longitudinal_accel, LongitudinalParams.ACCEL_LOOKUP, LongitudinalParams.TORQUE_LOOKUP))
      # Separate fields intentionally: equality is only the initial Elkoled approximation.
      self.longitudinal_potential_torque = torque
      self.longitudinal_wheel_torque = torque
      self.longitudinal_min_time = LongitudinalParams.MIN_TIME_GMP_EXPERIMENTAL
  # [psa longitudinal] - END

  def _reset_lat_state(self):
    self.status = 2
    self.apply_torque_factor = 0
    # self.takeover_req = 0
    self.last_status_change_frame = 0
    self.deactivation_in_progress = False
    self.eps_activation_frame = 0
    self.takeover_req_already_sent = False

  def _start_eps_active_cycle(self):
    self.eps_activation_frame = self.frame
    self.takeover_req_already_sent = False

  def _deactivate_eps(self):
    # Primo gradino della scaletta forzata. I due invii successivi salgono a 3 e 4
    # anche se CS.eps_active non e' ancora sceso.
    self.status = 2
    self.apply_torque_factor = 0
    self.eps_activation_frame = 0
    # self.takeover_req = 0
    self.last_status_change_frame = self.frame
    self.deactivation_in_progress = True

  def _activate_eps(self, CARSTATE, curvature):
    eps_active = CARSTATE.eps_active
    self.deactivation_in_progress = False
    self.eps_activation_frame = 0

    if not eps_active: # and not CS.out.steeringPressed:
      lateral_accel = abs(curvature) * CARSTATE.out.vEgo ** 2
      if not self.takeover_req_already_sent and lateral_accel >= self.params.EPS_ACTIVATE_TAKEOVER_FULL_LAT_ACCEL:
        self.takeover_req = 1
        self.takeover_req_already_sent = True

      self.status = 2 if self.status == 4 else self.status + 1

      # EPS likes a progressive activation of the Torque Factor
      self.apply_torque_factor += 10
      self.apply_torque_factor = min(self.apply_torque_factor, self.params.MAX_TORQUE_FACTOR)

  def _should_preempt_eps_rearm(self, v_ego, current_curvature):
    if self.model_sm is None or self.eps_activation_frame == 0:
      return False

    self.model_sm.update(0)
    if not (self.model_sm.seen['modelV2'] and self.model_sm.valid['modelV2'] and self.model_sm.alive['modelV2']):
      return False

    model = self.model_sm['modelV2']
    elapsed = (self.frame - self.eps_activation_frame) * DT_CTRL
    return should_preempt_eps_rearm(
      elapsed,
      v_ego,
      current_curvature,
      model.orientationRate.t,
      model.orientationRate.z,
      model.velocity.x,
    )

  def _maybe_request_eps_takeover(self, v_ego, current_curvature):
    if self.eps_activation_frame == 0:
      return

    model_valid = False
    model_t = ()
    model_yaw_rate = ()
    model_speed = ()
    if self.model_sm is not None:
      self.model_sm.update(0)
      model_valid = self.model_sm.seen['modelV2'] and self.model_sm.valid['modelV2'] and self.model_sm.alive['modelV2']
      if model_valid:
        model = self.model_sm['modelV2']
        model_t = model.orientationRate.t
        model_yaw_rate = model.orientationRate.z
        model_speed = model.velocity.x

    elapsed = (self.frame - self.eps_activation_frame) * DT_CTRL
    if should_request_eps_takeover(
      elapsed,
      v_ego,
      current_curvature,
      self.takeover_req_already_sent,
      model_valid,
      model_t,
      model_yaw_rate,
      model_speed,
      self.params.EPS_REARM_PERIOD,
    ):
      self.takeover_req = 1
      self.takeover_req_already_sent = True

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators
    self.apply_new_torque = 0
    # apply_new_torque = 0
    temp_driverSteeringTorque = 0
    new_torque_scaled = 0
    apply_new_torque_scaled = 0
    can_torque = 0

    # lateral control
    if self.CP.steerControlType == SteerControlType.torque:
      if self.frame % self.params.STEER_STEP == 0:
        if not CC.latActive:
          if self.latActiveLast:
             self.takeover_req = 1
          self._reset_lat_state()
        else:
          if not CS.eps_active:
            if self.eps_active_last and CS.speed_kph <= LKAS_LIMITS.DISABLE_SPEED:
              self.takeover_req = 1
            self._activate_eps(CS, actuators.curvature)

          else:
            # first time it enters in the lateral active state, store the frame to check the rearm period
            rearm_due = self.eps_activation_frame > 0 and self.frame >= self.eps_activation_frame + self.eps_rearm_frames
            rearm_before_curve = self._should_preempt_eps_rearm(CS.out.vEgo, actuators.curvature)
            if rearm_due or rearm_before_curve:
              self._deactivate_eps()
            elif self.deactivation_in_progress:
              self._deactivate_eps()
            else:
              ##########
              ### START EPS ACTIVE
              ######
              # EPS is active, proceed with lateral control
              if self.eps_activation_frame == 0:
                self._start_eps_active_cycle()
              self.takeover_req = 0
              self.status = 4 # 4: EPS ACTIVE
              self._maybe_request_eps_takeover(CS.out.vEgo, actuators.curvature)

              if (CS.out.steeringPressed):
                #### DRIVER STEERING DETECTED
                # If the driver is applying torque, give up the assist torque to avoid fighting the driver.
                self.apply_torque_factor = 0
                apply_new_torque_scaled = 0
                # apply_new_torque = 0
              else:
                actuatorsRequestedTorque = CC.actuators.torque * self.params.STEER_MAX
                ratio = min(1.0, (abs(actuatorsRequestedTorque) / float(self.params.STEER_MAX)) * 1.0) **1.2
                self.apply_torque_factor = int(self.params.MIN_TORQUE_FACTOR + ratio * (self.params.MAX_TORQUE_FACTOR - self.params.MIN_TORQUE_FACTOR))
                self.apply_torque_factor = max(self.params.MIN_TORQUE_FACTOR, min(self.apply_torque_factor, self.params.MAX_TORQUE_FACTOR))
                new_torque_scaled = int(round(actuatorsRequestedTorque * self.apply_torque_factor / 100))
                temp_driverSteeringTorque = CS.out.steeringTorque
                apply_new_torque_scaled = apply_driver_steer_torque_limits(new_torque_scaled, self.apply_torque_scaled_last,
                                                                temp_driverSteeringTorque, self.params, self.params.STEER_MAX)

        # if CC.latActive and CS.eps_active and self.frame % 500 in (0, 5, 10):
        #   apply_new_torque_scaled = 0
        #   self.apply_torque_factor = 0
        #   carlog.error(f"PSA_DEBUG sending empty torque apply_new_torque_scaled={apply_new_torque_scaled} ")

        # if CC.latActive and CS.eps_active and self.frame % 3000 in (0, 5, 10):
        #   self.takeover_req = 1

        if self.apply_torque_factor > 0 and apply_new_torque_scaled != 0:
          can_torque = int(round(apply_new_torque_scaled / self.apply_torque_factor *100))
        else:
          can_torque = 0
        # can_sends.append(create_lka_steering(self.packer, CC.latActive, can_torque, self.apply_torque_factor, self.status))
        can_sends.append(create_lka_steering(self.packer, CC.latActive, can_torque, self.apply_torque_factor, self.status)) #,apply_new_torque_scaled))
        # Remember the effective (scaled) value for the next frame's rate limit.
        self.apply_torque_scaled_last = apply_new_torque_scaled
        self.apply_can_torque_last = can_torque
        ### END EPS ACTIVE
        ##########

    # if self.car_fingerprint in (CAR.PSA_PEUGEOT_3008,):
    #   if self.frame % 10 == 0:
    #     # send steering wheel hold message
    #     can_sends.append(create_steering_hold(self.packer, CC.latActive, CS.is_dat_dira))

    #  ELKOLED LONGITUDINAL CONTROL

    # TUNING
    # >=-0.5: Engine brakes only
    # <-0.5: Add friction brakes
    # pitch = CC.orientationNED[1] if len(CC.orientationNED) == 3 else 0.0
    # accel_slope = math.sin(pitch) * 9.81
    # accel_cmd = actuators.accel + accel_slope

    # brake_accel = -0.5

    # # torque lookup
    # ACCEL_LOOKUP = [-1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0]
    # TORQUE_LOOKUP = [-400, -300, 120, 350, 550, 800, 1000]

    # # calculate Torque
    # torque_nm = interp(accel_cmd, ACCEL_LOOKUP, TORQUE_LOOKUP)
    # torque = max(-400, min(torque_nm, 1000))

    # braking = accel_cmd < brake_accel and not CS.out.gasPressed
    # if self.CP.openpilotLongitudinalControl:
    #   if CC.hudControl.leadVisible:
    #     sm.update(0)
    #     leads_v3 = sm['modelV2'].leadsV3
    #     if leads_v3 and leads_v3[0].x:
    #       r = leads_v3[0].x[0] / (5 + CS.out.vEgo)
    #       if self.bars > 3:  # initialize from "no lead"
    #         self.bars = min(3, int(r))
    #       elif r > self.bars + 1.2:
    #         self.bars = min(3, self.bars + 1)
    #       elif r < self.bars - 0.2:
    #         self.bars = max(0, self.bars - 1)
    #   else:
    #     self.bars = 4

    #   # disable radar ECU by setting to programming mode
    #   if self.radar_disabled == 0:
    #     can_sends.append(create_disable_radar())
    #     self.radar_disabled = 1

    #   # keep radar ECU disabled by sending tester present
    #   if self.frame % 100 == 0 and self.frame>0: # TODO check if disable_radar is sent 100 frames before
    #     can_sends.append(make_tester_present_msg(0x6b6, 1, suppress_response=False))

    #   # Highest torque seen without gas input: ~1000
    #   # Lowest torque seen without break mode: -560 (but only when transitioning from brake to accel mode, else -248)
    #   # Lowest brake mode accel seen: -4.85m/s²

    #   if self.frame % 2 == 0:
    #     can_sends.append(create_HS2_DYN1_MDD_ETAT_2B6(
    #       self.packer, self.frame // 2, actuators.accel, CS.out.cruiseState.enabled,
    #       CS.out.gasPressed, braking, CS.out.brakePressed, CS.out.standstill, torque,
    #     ))
    #     can_sends.append(create_HS2_DYN_MDD_ETAT_2F6(self.packer, braking, CC.hudControl.leadVisible, self.bars))

    # # stock long
    # # emulate resume button every 3 seconds to prevent autohold timeout
    # elif CC.latActive and CS.out.standstill and CC.hudControl.leadVisible:
    #   # map: {frame:status} - 0, 1
    #   status = {0: 0, 5: 1}.get(self.frame % 300)
    #   if status is not None:
    #     msg = CS.hs2_dat_mdd_cmd_452
    #     counter = (msg['COUNTER'] + 1) % 16
    #     can_sends.append(create_resume_acc(self.packer, counter, status, msg))

    # #  ELKOLED LONGITUDINAL CONTROL

    # [artiv probe] - START
    # [neutral motion] - START
    # Start ARTIV substitution while parked, after acceptance and stock radar silence.
    # [neutral motion] - END
    if self.car_fingerprint == CAR.PSA_PEUGEOT_3008 and not self.artiv_programming_requested:
      if not CS.out.standstill or not CS.out.canValid:
        # Restart the wait when moving or CAN data is unavailable.
        self.artiv_probe_last_frame = self.frame
      elif self.frame - self.artiv_probe_last_frame >= int(ARTIV_PROGRAMMING_WAIT / DT_CTRL):
        can_sends.append(create_disable_radar())
        self.artiv_programming_requested = True
        self.neutral_radar.start(now_nanos)
    if self.neutral_radar is not None:
      self.neutral_radar.update(self.frame, now_nanos, CS.out.standstill, CS.out.canValid)
    # [psa longitudinal] - START
    self._update_longitudinal(CC, CS)
    # [psa longitudinal] - END
    if self.neutral_radar is not None:
      if self.neutral_radar.active:
        radar_frame = self.frame - self.neutral_radar.started_frame
        if radar_frame % 2 == 0:  # 50 Hz
          counter = (radar_frame // 2) % 16
          # [psa longitudinal] - START
          # Default profile retains the recorded neutral encodings. Only the experimental
          # profile with a confirmed session and authorized longActive may request actuation.
          can_sends.append(create_HS2_DYN1_MDD_ETAT_2B6(
            self.packer, PSA_ADAS_BUS,
            mdd_desired_deceleration=self.longitudinal_accel if self.longitudinal_braking else LongitudinalParams.INACTIVE_ACCEL,
            potential_wheel_torque_request=(2 if self.longitudinal_braking else 1) if self.longitudinal_active else 0,
            min_time_for_desired_gear=self.longitudinal_min_time,
            gmp_potential_wheel_torque=self.longitudinal_potential_torque,
            acc_status=4 if self.longitudinal_active else 2,
            gmp_wheel_torque=self.longitudinal_wheel_torque,
            wheel_torque_request=int(self.longitudinal_active and not self.longitudinal_braking),
            auto_braking_status=3,
            mdd_decel_type=int(self.longitudinal_braking),
            mdd_decel_control_req=int(self.longitudinal_braking),
            gear_type=counter & 1,  # observed alternating bit; its DBC name is unverified
            prefill_request=0,
            counter=counter,
          ))
          can_sends.append(create_HS2_DYN_MDD_ETAT_2F6(
            self.packer, PSA_ADAS_BUS,
            target_detected=0,
            request_takeover=self.takeover_req if self.longitudinal_enabled else 0,
            blind_sensor=0,
            req_visual_coll_alert_arc=0,
            req_audio_coll_alert_arc=0,
            req_haptic_coll_alert_arc=0,
            inter_vehicle_distance=255.5,
            arc_status=6,
            auto_braking_in_progress=0,
            aeb_enabled=0,
            drive_away_request=0,
            display_intervehicle_time=6.2,
            mdd_decel_control_req=int(self.longitudinal_braking),
            auto_braking_status=3,
            counter=counter,
            target_position=0,
          ))
          # The one periodic 0x2F6 also owns lateral takeover, including when longActive is false.
          if self.longitudinal_enabled and self.takeover_req > 0:
            self.start_takeover_repeats += 1
            if self.start_takeover_repeats >= 2:
              self.takeover_req = 0
              self.start_takeover_repeats = 0
          # [psa longitudinal] - END
        if radar_frame % 10 == 0:  # 10 Hz
          can_sends.append(create_HS2_DAT_ARTIV_V2_4F6(
            self.packer, PSA_ADAS_BUS,
            time_gap=25.5, distance_gap=254, relative_speed=93.8,  # recorded no-target sentinels
            artiv_sensor_state=2, target_detected=0, artiv_target_change_info=0, traffic_direction=0,
          ))
        if radar_frame % 100 == 0:  # 1 Hz
          can_sends.append(create_HS2_SUPV_ARTIV_796(
            self.packer, PSA_ADAS_BUS,
            fault_code=0, status_no_config=0, status_partial_wakeup_gmp=0, uce_electr_state=0,
          ))
          if radar_frame > 0:
            can_sends.append(CanData(0x6B6, b'\x02\x3e\x00', PSA_ADAS_BUS))
            self.neutral_radar.last_tester_present_nanos = now_nanos
    # [artiv probe] - END

    if self.car_fingerprint in (CAR.PSA_PEUGEOT_3008,CAR.PSA_CITROEN_C4_SPACETOURER):
      # # Keep requesting the ARTIV programming session. A single request can be
      # # missed or rejected while the ECU/gateway is still initializing.
      # if not self.radar_disabled and self.frame > 200:
      #   can_sends.append(create_disable_radar())
      #   self.radar_disabled = True

      # # Keep the diagnostic session alive halfway between programming requests,
      # # avoiding two UDS requests in the same control frame.
      # if self.frame % 100 == 50 and self.radar_disabled:
      #   can_sends.append(make_tester_present_msg(0x6b6, PSA_ADAS_BUS, suppress_response=False))

      if not CC.latActive:
        self.steering_hold_counter = 0                       # alla ripresa il primo
        self.next_steering_hold = random.randint(8, 12)      # hold-hands parte subito
        self.driver_torque_counter = 0
        self.next_driver_torque = random.randint(500, 800)
      # [CLAUDE stop-finti-durante-riarmo] - END
      else:
        # --- HOLD HANDS (~10 Hz con jitter 8–12 frame) ---
        self.steering_hold_counter += 1
        if self.steering_hold_counter >= self.next_steering_hold:
          can_sends.append(create_steering_hold(self.packer, CC.latActive, CS.is_dat_dira))
          self.steering_hold_counter = 0
          self.next_steering_hold = random.randint(8, 12)
        # --- DRIVER TORQUE (ogni 5–8 s) ---
        # self.driver_torque_counter += 1
        # if self.driver_torque_counter >= self.next_driver_torque:
        #   msg = CS.steering
        #   counter = (msg['COUNTER'] + 1) % 16
        #   # can_sends.append(create_driver_torque(self.packer, CS.steering, counter))
        #   self.driver_torque_counter = 0
        #   self.next_driver_torque = random.randint(500, 800)

    # if self.car_fingerprint in (CAR.PSA_PEUGEOT_3008,CAR.PSA_CITROEN_C4_SPACETOURER):
    #   if CC.enabled and CS.out.vEgo < self.params.RESUME_ACC_SPEED and CC.hudControl.leadVisible:
    #     if self.creep_start_frame == 0:
    #       self.creep_start_frame = self.frame     # primo frame dentro la finestra
    #     phase = (self.frame - self.creep_start_frame) % 300
    #     if phase in (0, 5):
    #       pressed = 1 if phase == 5 else 0
    #       msg = CS.hs2_dat_mdd_cmd_452
    #       counter = (msg['COUNTER'] + 1) % 16
    #       can_sends.append(create_resume_acc(self.packer, counter, pressed, msg))
    #   else:
    #     self.creep_start_frame = 0

    if self.car_fingerprint in (CAR.PSA_PEUGEOT_3008,CAR.PSA_CITROEN_C4_SPACETOURER):
      # The neutral trial owns 0x2F6 after its request; never mix in model/lateral alerts.
      neutral_trial_requested = self.neutral_radar is not None and self.neutral_radar.request_nanos is not None
      if self.takeover_req > 0 and self.frame % 2 == 0 and not neutral_trial_requested: # 50 Hz
        self.start_takeover_repeats +=1
        # if self.takeover_start_msg_frame == 0:
        #   self.takeover_start_msg_frame = self.frame
        can_sends.append(create_request_takeover(self.packer, CS.HS2_DYN_MDD_ETAT_2F6,self.takeover_req))
        # carlog.error("PSA_DEBUG sending to CAN create_request_takeover")
        # if self.frame > self.takeover_start_msg_frame + self.takeover_msg_duration: # 1 s
        # carlog.error("PSA_DEBUG takeover_req = False")
        if self.start_takeover_repeats > 1:
          self.takeover_req = 0
          self.start_takeover_repeats = 0

        # self.takeover_start_msg_frame = 0

    # if self.CP.openpilotLongitudinalControl and CC.enabled:
    #   # Disable ARTIV only for full openpilot longitudinal. ICBM deliberately
    #   # leaves ARTIV active: the stock controller executes our dynamic setpoint.
    #   if self.radar_disabled == 0:
    #     can_sends.append(create_disable_radar())
    #     self.radar_disabled = 1

    #   if self.frame % 100 == 0 and self.frame > 0:
    #     can_sends.append(make_tester_present_msg(0x6b6, 1, suppress_response=False))

    # Actuators output
    new_actuators = actuators.as_builder()
    # [psa longitudinal] - START
    if self.longitudinal_profile:
      new_actuators.accel = self.longitudinal_accel
    # [psa longitudinal] - END
    if self.CP.steerControlType == SteerControlType.torque:
      # Keep last applied torque between 20 Hz LKA updates.
      # The EPS maintains assist longer than 50 ms, preventing gaps in actuator output.
      new_actuators.torque = self.apply_torque_scaled_last / self.params.STEER_MAX
      new_actuators.torqueOutputCan = self.apply_torque_scaled_last
      # new_actuators.steeringAngleDeg = float(self.apply_can_torque_last)
      # new_actuators.curvature = temp_driverSteeringTorque   # lo vedi in juggle come carControl.actuatorsOutput.curvature
      # new_actuators.steeringAngleDeg = float(self.apply_torque_factor)

      # if self.frame % 100 == 0:
      #   carlog.error(f"PSA_DEBUG torque={new_actuators.torque:.3f} torque_can={self.apply_torque_scaled_last}")
    if self.frame % self.params.STEER_STEP == 0:
      self.latActiveLast = CC.latActive
      self.eps_active_last = CS.eps_active

    self.frame += 1
    return new_actuators, can_sends
