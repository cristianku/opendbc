from opendbc.can.packer import CANPacker
# [CLAUDE eps-rearm] - START
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, structs, DT_CTRL
# [CLAUDE eps-rearm] - END
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.can_definitions import CanData
from opendbc.car.psa.psacan import (
  # create_driver_torque,
  create_lka_steering,
  create_request_takeover,
  create_resume_acc,
  create_steering_hold,
  create_disable_radar,
  create_HS2_DYN1_MDD_ETAT_2B6,
  create_HS2_DYN_MDD_ETAT_2F6,
  create_HS2_DAT_ARTIV_V2_4F6,
  create_HS2_SUPV_ARTIV_796,
  
)
from opendbc.car.psa.values import CarControllerParams, CAR, LKAS_LIMITS, PSA_ADAS_BUS
from opendbc.car.carlog import carlog
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
ARTIV_PROGRAMMING_WAIT = 2.0  # seconds of valid CAN before the one-shot request, including while moving
# [artiv probe] - END
RADAR_IDS = (0x2B6, 0x2F6, 0x4F6, 0x796)
RADAR_TX_TIMEOUTS = {0x2B6: 250_000_000, 0x2F6: 250_000_000, 0x4F6: 500_000_000, 0x796: 2_000_000_000}


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
    self.packer = CANPacker(dbc_names[Bus.main])
    self.apply_torque_scaled_last = 0
    self.apply_torque_factor = 0
    self.status = 2
    self.takeover_req = 0
    self.start_takeover_repeats = 0
    # Shared latch for both the pre-rearm warning and an immediate curve warning during EPS reactivation.
    self.takeover_req_already_sent = False
    self.model_sm = messaging.SubMaster(['modelV2']) if messaging is not None else None

    # this is the frame when the latactive is being pressed
    self.car_fingerprint = CP.carFingerprint
    self.params = CarControllerParams(CP)
    # [psa longitudinal] - START
    self.longitudinal_profile = self.car_fingerprint in (CAR.PSA_PEUGEOT_3008,CAR.PSA_CITROEN_C4_SPACETOURER) and CP.openpilotLongitudinalControl
    self.longitudinal_enabled = (self.longitudinal_profile and not CP.dashcamOnly and not CP.passive
                                 and any(c.safetyModel == structs.CarParams.SafetyModel.psa and c.safetyParam & PSA_LONG_CONTROL
                                         for c in CP.safetyConfigs))
    # [acc hold] - START
    self.longitudinal_active = False
    self.acc_on_hold = False
    # [acc hold] - END
    # [light braking] - START
    self.longitudinal_braking = False
    # [light braking] - END
    # [long flow] - START
    # Inactive values also exist when openpilot longitudinal is disabled.
    self.longitudinal_accel = 0.0

    # Last acceleration actually allowed to reach the vehicle.
    # Used to limit positive jerk without delaying braking.
    self.longitudinal_accel_limited = 0.0

    self.longitudinal_potential_torque = LongitudinalParams.INACTIVE_TORQUE

    self.longitudinal_wheel_torque = LongitudinalParams.INACTIVE_TORQUE
    self.longitudinal_min_time = 0.0
    # [long flow] - END
    # [psa longitudinal] - END
    # [artiv probe] - START
    self.artiv_programming_requested = False
    self.artiv_probe_last_frame = 0
    # [neutral motion] - START
    # Allow session startup and continued substitutes/TesterPresent while moving, including in reverse.
    # Actuation remains gated separately by longitudinal_enabled in _update_longitudinal.
    self.radar_request_nanos = None
    self.radar_accepted_nanos = None
    self.radar_started_nanos = None
    self.radar_started_frame = None
    self.radar_last_rx_nanos = None
    self.radar_last_bus_nanos = 0
    self.radar_last_diag_reply_nanos = 0
    self.radar_last_tester_present_nanos = None
    self.radar_last_echo_nanos = {}
    self.radar_active = False
    self.radar_stop_reason = None
    # [neutral motion] - END
    # [artiv probe] - END
    # [lead display] - START
    # Fasce Elkoled: r = distanza [m] / (5 + velocita [m/s]); non sono metri fissi.
    # Alla comparsa del target: 0 = r < 1, 1 = 1 <= r < 2, 2 = 2 <= r < 3, 3 = r >= 3.
    # Esempio a 36 km/h (10 m/s): 0 = 0-15 m, 1 = 15-30 m, 2 = 30-45 m, 3 = >=45 m
    # (estremo superiore escluso). Poi l'isteresi cambia fascia oltre bars+1.2 o sotto bars-0.2.
    # 4 = nessun target/dato non valido: stato interno, NON una fascia "piu distante".
    # Sul CAN: target presente -> TARGET_POSITION 0..3; assente -> TARGET_DETECTED=0, POSITION=0.
    # La corrispondenza grafica delle posizioni sul quadro resta da verificare sulla vettura.
    self.bars = 4
    # [lead display] - END
    self.steering_hold_counter = 0
    self.next_steering_hold = random.randint(8, 12)  # ~10Hz con jitter ±20%
    self.last_activation_frame = 0
    self.eps_activation_frame = 0
    self.creep_start_frame = 0
    self.deactivation_in_progress = False
    self.eps_rearm_frames = int(self.params.EPS_REARM_PERIOD / DT_CTRL)
    self.takeover_msg_duration = int(self.params.TAKEOVER_MSG_DURATION / DT_CTRL)   # 0.1 s = 10 frame

  def _stop_radar_session(self, reason):
    self.radar_active = False
    if self.radar_stop_reason is None:
      self.radar_stop_reason = reason
      carlog.warning('ARTIV session: stopped (%s); no automatic retry', reason)

  def process_radar_can(self, can_packets):
    if self.car_fingerprint not in (CAR.PSA_PEUGEOT_3008,CAR.PSA_CITROEN_C4_SPACETOURER):
      return can_packets
    # Inspect all genuine RX first, so a radar return stops echo remapping even if an echo
    # precedes it within this batch. src 129 is a TX receipt; src 193 is a blocked TX.
    for nanos, messages in can_packets:
      for address, data, src in messages:
        if src != PSA_ADAS_BUS:
          continue
        self.radar_last_bus_nanos = max(self.radar_last_bus_nanos, nanos)
        if address in RADAR_IDS:
          self.radar_last_rx_nanos = nanos
          if self.radar_active and nanos >= self.radar_started_nanos:
            self._stop_radar_session('stock radar resumed')
        if (address != 0x696 or self.radar_request_nanos is None or nanos <= self.radar_request_nanos
            or self.radar_stop_reason is not None or len(data) < 3):
          continue
        size = data[0]
        if not 2 <= size <= 7 or len(data) < size + 1:
          continue  # only complete ISO-TP single frames, never stale/multiframe fields
        if data[1:3] == b'\x50\x02' and size == 6 and self.radar_accepted_nanos is None:
          self.radar_accepted_nanos = nanos
          self.radar_last_diag_reply_nanos = nanos
        elif (data[1:3] == b'\x7e\x00' and size == 2 and self.radar_active
              and self.radar_last_tester_present_nanos is not None and nanos > self.radar_last_tester_present_nanos):
          self.radar_last_diag_reply_nanos = nanos
        elif size == 3 and data[1] == 0x7F and data[2] in (0x10, 0x3E) and data[3] != 0x78:
          self._stop_radar_session(f'diagnostic refusal {data[2]:02x}/{data[3]:02x}')

    if not self.radar_active:
      return can_packets
    result = []
    for nanos, messages in can_packets:
      received = []
      for address, data, src in messages:
        if src == PSA_ADAS_BUS + 128 and address in RADAR_IDS and nanos >= self.radar_started_nanos:
          self.radar_last_echo_nanos[address] = nanos
          src = PSA_ADAS_BUS
        received.append(CanData(address, data, src))
      result.append((nanos, received))
    return result

  def _update_radar_session(self, now_nanos, can_valid):
    if self.radar_request_nanos is None or self.radar_stop_reason is not None:
      return
    if not self.radar_active:
      if now_nanos - self.radar_request_nanos > 1_000_000_000:
        self._stop_radar_session('no confirmed silent radar within 1 s')
        return
      # [radar handover] - START
      # process_can inspects all genuine RX before update. Start on confirmation without
      # an extra silence timer, unless stock frames were received at or after that reply.
      # Equal timestamps cannot establish ordering within a CAN packet, so also block.
      if (self.radar_accepted_nanos is None or self.radar_last_rx_nanos is None
          or self.radar_last_rx_nanos >= self.radar_accepted_nanos):
        return
      # [radar handover] - END
      if not can_valid:
        self._stop_radar_session('vehicle CAN invalid before emulation')
        return
      self.radar_active = True
      self.radar_started_nanos = now_nanos
      self.radar_started_frame = self.frame
      # [neutral motion] - START
      carlog.info('ARTIV: emulation started (motion allowed)')
      # [neutral motion] - END

    if now_nanos - self.radar_last_bus_nanos > 250_000_000:
      self._stop_radar_session('ADAS bus RX timeout')
    elif any(now_nanos - self.radar_last_echo_nanos.get(addr, self.radar_started_nanos) > timeout for addr, timeout in RADAR_TX_TIMEOUTS.items()):
      self._stop_radar_session('radar TX echo timeout')
    elif now_nanos - self.radar_started_nanos > 250_000_000 and not can_valid:
      # Allow the first real echoes/counters to settle, then require all vehicle CAN,
      # including wheel speed and brake buses, rather than trusting a stale standstill.
      self._stop_radar_session('vehicle CAN invalid')
    elif now_nanos - self.radar_last_diag_reply_nanos > 2_000_000_000:
      self._stop_radar_session('TesterPresent response timeout')

  def _update_longitudinal(self, CC, CS):
    """Prepare explicit CAN inputs. Experimental torque mapping; no emission or scheduling here."""
    # Preserve braking hysteresis, then clear the previous cycle's requests.
    was_braking = self.longitudinal_braking
    self.acc_on_hold = False
    self.longitudinal_active = False
    self.longitudinal_braking = False
    self.longitudinal_accel = 0.0
    self.longitudinal_potential_torque = LongitudinalParams.INACTIVE_TORQUE
    self.longitudinal_wheel_torque = LongitudinalParams.INACTIVE_TORQUE
    self.longitudinal_min_time = 0.0

    # Configuration and radar session must permit experimental control.
    if not self.longitudinal_enabled or not self.radar_active:
      self.longitudinal_accel_limited = 0.0
      return

    # Require valid CAN, Sunnypilot enabled, BSI consent and no brake pedal.
    if not CS.out.canValid or not CC.enabled or not CS.out.cruiseState.enabled or CS.out.brakePressed:
      self.longitudinal_accel_limited = 0.0
      return

    # Gas temporarily suspends ACC only while Sunnypilot and BSI remain enabled.
    if CS.out.gasPressed:
      self.acc_on_hold = True
      self.longitudinal_accel_limited = 0.0
      return

    if not CC.longActive or not math.isfinite(CC.actuators.accel):
      self.longitudinal_accel_limited = 0.0
      return

    # accel = max(LongitudinalParams.ACCEL_LOOKUP[0], min(CC.actuators.accel, LongitudinalParams.ACCEL_LOOKUP[-1]))

    requested_accel = max(
      LongitudinalParams.ACCEL_LOOKUP[0],
      min(CC.actuators.accel, LongitudinalParams.ACCEL_LOOKUP[-1]),
    )

    # Limit only increasing acceleration.
    # Braking/deceleration must remain immediately available.
    if requested_accel > self.longitudinal_accel_limited:
      # Never slowly ramp through negative acceleration when transitioning
      # from braking/coast to throttle: release braking immediately to zero.
      accel_base = max(self.longitudinal_accel_limited, 0.0)

      accel = min(
        requested_accel,
        accel_base + LongitudinalParams.POSITIVE_JERK_MAX * DT_CTRL
      )
    else:
      # Falling acceleration, including emergency braking, is unrestricted.
      accel = requested_accel

    # Keep the service brake through light deceleration and speed holding. Reset
    # above on every update so pedals, disengagement and invalid accel/CAN clear it.
    braking = accel < LongitudinalParams.BRAKE_ENTER_ACCEL or (was_braking and accel <= 0.0)
    pitch = 0.0  # No orientation supplied: use the level-road map.
    # if not braking and len(CC.orientationNED) == 3:
    #   pitch = CC.orientationNED[1]
    #   if not math.isfinite(pitch):
    #     return
    if not braking and len(CC.orientationNED) == 3:
      pitch = CC.orientationNED[1]
      if not math.isfinite(pitch):
        self.longitudinal_accel_limited = 0.0
        return

    equivalent_accel = accel + ACCELERATION_DUE_TO_GRAVITY * math.sin(pitch)
    # Use the existing provisional GMP/brake crossover with grade compensation
    # for entry too: on a descent a light/zero target can require service braking.
    # A positive vehicle-acceleration request always leaves the brake path.
    braking |= accel <= 0.0 and equivalent_accel < LongitudinalParams.BRAKE_ENTER_ACCEL

    if braking:
      accel = max(LongitudinalParams.BRAKE_MIN_ACCEL, min(CC.actuators.accel, 0.0))

    # Remember the acceleration actually applied to the vehicle.
    self.longitudinal_accel_limited = accel

    self.longitudinal_active = True
    self.longitudinal_accel = accel
    self.longitudinal_braking = braking
    if not self.longitudinal_braking:
      # Compensate the GMP map only: the brake ECU already takes a deceleration request.
      # interp saturates to the existing provisional endpoints (-400..1000 Nm).
      self.longitudinal_potential_torque = float(interp(equivalent_accel, LongitudinalParams.ACCEL_LOOKUP,
                                                       LongitudinalParams.POTENTIAL_TORQUE_LOOKUP))
      self.longitudinal_wheel_torque = float(interp(equivalent_accel, LongitudinalParams.ACCEL_LOOKUP,
                                                   LongitudinalParams.TORQUE_LOOKUP))
      self.longitudinal_min_time = LongitudinalParams.MIN_TIME_GMP_EXPERIMENTAL

  def _update_lead_display(self, CC, CS):
    """Select the cluster target position using Elkoled's distance/speed heuristic."""
    previous_bars = self.bars
    self.bars = 4  # internal no-target sentinel; restart the bucket when a lead returns
    if not CC.hudControl.leadVisible or self.model_sm is None:
      return False

    self.model_sm.update(0)
    if not (self.model_sm.seen['modelV2'] and self.model_sm.valid['modelV2'] and self.model_sm.alive['modelV2']):
      return False
    leads = self.model_sm['modelV2'].leadsV3
    if not leads or not leads[0].x:
      return False
    distance = leads[0].x[0]
    denominator = 5 + CS.out.vEgo
    if not (math.isfinite(distance) and distance >= 0 and math.isfinite(denominator) and denominator > 0):
      return False
    ratio = distance / denominator
    if not math.isfinite(ratio):
      return False

    if previous_bars > 3:
      self.bars = min(3, int(ratio))
    elif ratio > previous_bars + 1.2:
      self.bars = min(3, previous_bars + 1)
    elif ratio < previous_bars - 0.2:
      self.bars = max(0, previous_bars - 1)
    else:
      self.bars = previous_bars
    # | `self.bars` | Distanza |
    # |---|---|
    # | `0` | Meno di 15 m |
    # | `1` | Da 15 a meno di 30 m |
    # | `2` | Da 30 a meno di 45 m |
    # | `3` | 45 m o più |
    # | `4` | **Nessun target o dati non validi** |
    self.bars = 2
    return True

  def _reset_lat_state(self):
    self.status = 2
    self.apply_torque_factor = 0
    # self.takeover_req = 0
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
    # apply_new_torque = 0
    temp_driverSteeringTorque = 0
    new_torque_scaled = 0
    apply_new_torque_scaled = 0
    can_torque = 0

    # lateral control
    lat_active = CC.latActive
    if self.car_fingerprint in (CAR.PSA_PEUGEOT_3008, CAR.PSA_CITROEN_C4_SPACETOURER):
      # Gate locally too: latActive can lag the raw wheel-speed threshold.
      # Drop float32 conversion noise at exactly 51 km/h (wheel resolution: 0.0025).
      lat_active = lat_active and round(CS.speed_kph, 4) > LKAS_LIMITS.ENABLE_SPEED
    if self.CP.steerControlType == SteerControlType.torque:
      if self.frame % self.params.STEER_STEP == 0:
        if not lat_active:
          if self.latActiveLast:
             self.takeover_req = 1
          self._reset_lat_state()
        else:
          if not CS.eps_active:
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

              # Match Panda before the filtered/debounced driver flag catches up.
              raw_driver_torque = getattr(CS, 'steering', {}).get('DRIVER_TORQUE', 0) * 3
              if CS.out.steeringPressed or abs(raw_driver_torque) > self.params.STEER_DRIVER_ALLOWANCE:
                #### DRIVER STEERING DETECTED
                # If the driver is applying torque, give up the assist torque to avoid fighting the driver.
                self.apply_torque_factor = 0
                apply_new_torque_scaled = 0
              else:
                actuatorsRequestedTorque = CC.actuators.torque * self.params.STEER_MAX
                ratio = min(1.0, (abs(actuatorsRequestedTorque) / float(self.params.STEER_MAX)) * 1.0) **1.2
                self.apply_torque_factor = int(self.params.MIN_TORQUE_FACTOR + ratio * (self.params.MAX_TORQUE_FACTOR - self.params.MIN_TORQUE_FACTOR))
                self.apply_torque_factor = max(self.params.MIN_TORQUE_FACTOR, min(self.apply_torque_factor, self.params.MAX_TORQUE_FACTOR))
                new_torque_scaled = int(round(actuatorsRequestedTorque * self.apply_torque_factor / 100))
                temp_driverSteeringTorque = CS.out.steeringTorque
                apply_new_torque_scaled = apply_driver_steer_torque_limits(new_torque_scaled, self.apply_torque_scaled_last,
                                                                temp_driverSteeringTorque, self.params, self.params.STEER_MAX)

        if self.apply_torque_factor > 0 and apply_new_torque_scaled != 0:
          can_torque = int(round(apply_new_torque_scaled / self.apply_torque_factor *100))
        else:
          can_torque = 0
        unknown2 = 24
        if self.car_fingerprint in (CAR.PSA_PEUGEOT_3008,CAR.PSA_CITROEN_C4_SPACETOURER) and not lat_active:
          unknown2 = getattr(CS, 'stock_lka_unknown2', 24)
        can_sends.append(create_lka_steering(
          self.packer, lat_active, can_torque, self.apply_torque_factor, self.status, unknown2=unknown2,
        ))
        # Remember the effective (scaled) value for the next frame's rate limit.
        self.apply_torque_scaled_last = apply_new_torque_scaled

    # Run experimental radar and longitudinal management only when configured.
    # Keep running after ACC disengagement to clear requests and maintain the session.
    if self.CP.openpilotLongitudinalControl:
      # Only take over the stock radar when openpilot longitudinal is enabled.
      # Cruise engagement gates actuation separately; disengagement keeps the session alive.
      if self.longitudinal_enabled and not self.artiv_programming_requested:
        if not CS.out.canValid:
          # Restart the wait only when CAN data is unavailable.
          self.artiv_probe_last_frame = self.frame
        elif self.frame - self.artiv_probe_last_frame >= int(ARTIV_PROGRAMMING_WAIT / DT_CTRL):
          can_sends.append(create_disable_radar())
          self.artiv_programming_requested = True
          self.radar_request_nanos = now_nanos
          carlog.info('ARTIV session: programming requested; waiting for 50 02 and radar silence')

      self._update_radar_session(now_nanos, CS.out.canValid)
      self._update_longitudinal(CC, CS)
      if self.radar_active:
        radar_frame = self.frame - self.radar_started_frame
        if radar_frame % 2 == 0:  # 50 Hz
          counter = (radar_frame // 2) % 16
          # Temporarily restore route 45's no-target display for radar fault diagnosis.
          lead_detected = self._update_lead_display(CC, CS)
          lead_detected = False
          # Default profile retains the recorded neutral encodings. Only the experimental
          # profile with a confirmed session and authorized longActive may request actuation.
          acc_waiting = not CS.out.brakePressed and CS.out.vEgoRaw >= self.CP.minEnableSpeed
          if self.acc_on_hold:
            acc_status = 5  # Suspended by accelerator pedal
          elif self.longitudinal_active:
            acc_status = 4  # Active control
          elif acc_waiting:
            acc_status = 3  # Ready for BSI activation
          else:
            acc_status = 2  # Inhibited
          # [long flow] - END
          can_sends.append(create_HS2_DYN1_MDD_ETAT_2B6(
            self.packer, PSA_ADAS_BUS,
            mdd_desired_deceleration=self.longitudinal_accel if self.longitudinal_braking else LongitudinalParams.INACTIVE_ACCEL,
            potential_wheel_torque_request=(2 if self.longitudinal_braking else 1) if self.longitudinal_active else 0,
            min_time_for_desired_gear=self.longitudinal_min_time,
            gmp_potential_wheel_torque=self.longitudinal_potential_torque,
            # Stock radar announces Waiting before the BSI requests ACC activation.
            # Readiness does not authorize torque or braking requests.
            acc_status=acc_status,
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
            target_detected=int(lead_detected),
            # target_detected=0,
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
            target_position=self.bars if lead_detected else 0,
          ))
          # The one periodic 0x2F6 also owns lateral takeover, including when longActive is false.
          if self.longitudinal_enabled and self.takeover_req > 0:
            self.start_takeover_repeats += 1
            if self.start_takeover_repeats >= 2:
              self.takeover_req = 0
              self.start_takeover_repeats = 0
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
            self.radar_last_tester_present_nanos = now_nanos
      # [artiv probe] - END
    # [long flow] - END

    if self.car_fingerprint in (CAR.PSA_PEUGEOT_3008,CAR.PSA_CITROEN_C4_SPACETOURER):
      if not lat_active:
        self.steering_hold_counter = 0                       # alla ripresa il primo
        self.next_steering_hold = random.randint(8, 12)      # hold-hands parte subito
      else:
        # --- HOLD HANDS (~10 Hz con jitter 8–12 frame) ---
        self.steering_hold_counter += 1
        if self.steering_hold_counter >= self.next_steering_hold:
          can_sends.append(create_steering_hold(self.packer, lat_active, CS.is_dat_dira))
          self.steering_hold_counter = 0
          self.next_steering_hold = random.randint(8, 12)

    if self.car_fingerprint in (CAR.PSA_PEUGEOT_3008,CAR.PSA_CITROEN_C4_SPACETOURER):
      if CC.enabled and CS.out.vEgo < self.params.RESUME_ACC_SPEED and CC.hudControl.leadVisible:
        if self.creep_start_frame == 0:
          self.creep_start_frame = self.frame     # primo frame dentro la finestra
        phase = (self.frame - self.creep_start_frame) % 300
        if phase in (0, 5):
          pressed = 1 if phase == 5 else 0
          msg = CS.hs2_dat_mdd_cmd_452
          counter = (msg['COUNTER'] + 1) % 16
          can_sends.append(create_resume_acc(self.packer, counter, pressed, msg))
      else:
        self.creep_start_frame = 0

    if self.car_fingerprint in (CAR.PSA_PEUGEOT_3008,CAR.PSA_CITROEN_C4_SPACETOURER):
      # The radar session owns 0x2F6 after its request; avoid separate takeover frames.
      radar_session_requested = self.radar_request_nanos is not None
      if self.takeover_req > 0 and self.frame % 2 == 0 and not radar_session_requested: # 50 Hz
        self.start_takeover_repeats +=1
        can_sends.append(create_request_takeover(self.packer, CS.HS2_DYN_MDD_ETAT_2F6,self.takeover_req))
        if self.start_takeover_repeats > 1:
          self.takeover_req = 0
          self.start_takeover_repeats = 0

    # Actuators output
    new_actuators = actuators.as_builder()
    if self.longitudinal_profile:
      new_actuators.accel = self.longitudinal_accel
    if self.CP.steerControlType == SteerControlType.torque:
      # Keep last applied torque between 20 Hz LKA updates.
      # The EPS maintains assist longer than 50 ms, preventing gaps in actuator output.
      new_actuators.torque = self.apply_torque_scaled_last / self.params.STEER_MAX
      new_actuators.torqueOutputCan = self.apply_torque_scaled_last

    if self.frame % self.params.STEER_STEP == 0:
      self.latActiveLast = lat_active

    self.frame += 1
    return new_actuators, can_sends
