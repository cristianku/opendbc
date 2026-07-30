from opendbc.can.packer import CANPacker
# [CLAUDE eps-rearm] - START
# from opendbc.car import Bus, structs, make_tester_present_msg
from opendbc.car import Bus, structs, DT_CTRL
# [CLAUDE eps-rearm] - END
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.psa.psacan import (
  create_driver_torque,
  create_lka_steering,
  create_request_takeover,
  create_resume_acc,
  create_steering_hold)
from opendbc.car.psa.values import CarControllerParams, CAR, LKAS_LIMITS
from opendbc.sunnypilot.car.psa.icbm import IntelligentCruiseButtonManagementInterface

# from cereal import messaging
# from numpy import interp

import random
# import math

SteerControlType = structs.CarParams.SteerControlType
ICBMState = structs.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState
# sm = messaging.SubMaster(['modelV2'], poll='modelV2')

class CarController(CarControllerBase, IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)
    self.latActiveLast = False
    self.eps_active_last = False
    self.packer = CANPacker(dbc_names[Bus.main])
    self.apply_torque_scaled_last = 0
    self.apply_can_torque_last = 0  # raw CAN torque logged to steeringAngleDeg (debug); init so it always exists
    self.apply_torque_factor = 0
    self.apply_torque = 0
    self.status = 2
    self.takeover_req = 0
    self.takeover_req_already_sent = False

    # this is the frame when the latactive is being pressed
    self.car_fingerprint = CP.carFingerprint
    self.params = CarControllerParams(CP)
    self.radar_disabled = 0
    self.bars = 4
    self.steering_hold_counter = 0
    self.next_steering_hold = random.randint(8, 12)  # ~10Hz con jitter ±20%
    self.driver_torque_counter = 0
    self.next_driver_torque = random.randint(500, 800)  # 5–8 s @100 Hz
    self.last_activation_frame = 0
    self.eps_activation_frame = 0
    self.activation_request_frame = 0
    # self.takeover_start_msg_frame = 0
    # [CLAUDE resume-acc-anticipato] - START
    # Frame di ingresso nella finestra di creep, creato QUI e non al primo uso
    # (attributo nato dentro update() = AttributeError se quel giro non parte per primo).
    self.creep_start_frame = 0
    # [CLAUDE resume-acc-anticipato] - END
    # [CLAUDE eps-rearm] - START
    # Stato della scaletta e dello stacco: tutti creati qui, mai al primo uso
    # (attributo nato dentro un metodo = AttributeError se quel metodo non gira per primo).
    self.last_status_change_frame = 0     # frame dell'ultimo cambio di gradino
    # True durante la scaletta forzata 2->3->4. Non aspetta piu' che l'EPS
    # confermi la disattivazione: ogni gradino dura un invio LKA (50 ms).
    self.deactivation_in_progress = False
    # Periodo di stacco EPS: da secondi a frame. self.frame gira a 100 Hz (DT_CTRL = 0.01 s),
    # quindi frame = secondi / DT_CTRL.
    self.eps_rearm_frames = int(self.params.EPS_REARM_PERIOD / DT_CTRL)   # 5 s = 500 frame
    # self.eps_activate_keep_status_frames = int(self.params.EPS_KEEP_STATUS_PERIOD / DT_CTRL)   # 0.1 s = 10 frame
    self.eps_activate_takeover_frames = int(self.params.EPS_ACTIVATE_TAKEOVER_PERIOD / DT_CTRL)   # 0.1 s = 10 frame
    # [CLAUDE eps-closed-loop] - START
    # Ultimo stato dell'EPS visto dalla scaletta: creato QUI, non al primo uso, se no
    # e' AttributeError al primo giro (stessa trappola di eps_rearm_failed).
    self.eps_state_last = 0
    # [CLAUDE eps-closed-loop] - END
    self.takeover_msg_duration = int(self.params.TAKEOVER_MSG_DURATION / DT_CTRL)   # 0.1 s = 10 frame
    # Riga rimossa: la scaletta conta frame (last_status_change_frame), non cicli LKA.
    # self.eps_status_hold_cycles = max(1, int(round(self.params.EPS_STATUS_HOLD / (DT_CTRL * self.params.STEER_STEP))))
    # [CLAUDE eps-rearm] - END

  def _reset_lat_state(self):
    self.status = 2
    self.apply_torque_factor = 0
    # self.takeover_req = 0
    self.last_status_change_frame = 0
    self.activation_request_frame = 0
    self.deactivation_in_progress = False
    self.eps_activation_frame = 0

  def _deactivate_eps(self):
    # Primo gradino della scaletta forzata. I due invii successivi salgono a 3 e 4
    # anche se CS.eps_active non e' ancora sceso.
    self.status = 2
    self.apply_torque_factor = 0
    self.eps_activation_frame = 0
    # self.takeover_req = 0
    self.last_status_change_frame = self.frame
    self.activation_request_frame = 0
    self.deactivation_in_progress = True
    self.takeover_req_already_sent = False

  def _activate_eps(self, CARSTATE, curvature):
    eps_active = CARSTATE.eps_active
    self.deactivation_in_progress = False
    self.eps_activation_frame = 0
    if self.activation_request_frame == 0:
      # first frame the EPS activate or re activate is sent
      self.activation_request_frame = self.frame
      # self.takeover_req_sent = 0
    lateral_accel = abs(curvature) * CARSTATE.out.vEgo ** 2
    curve_ratio = min(1.0, lateral_accel / 0.5)

    takeover_frames = round(
      self.eps_activate_takeover_frames * (1.0 - curve_ratio)
    )
    if not self.takeover_req_already_sent:
      if self.frame >= self.activation_request_frame + takeover_frames:
        # carlog.error("PSA_DEBUG _activate_eps - too long to activate - self.takeover_req = True")
        self.takeover_req = 2
        self.takeover_req_already_sent = True

    if not eps_active: # and not CS.out.steeringPressed:
      self.status = 2 if self.status == 4 else self.status + 1

      # EPS likes a progressive activation of the Torque Factor
      self.apply_torque_factor += 10
      self.apply_torque_factor = min(self.apply_torque_factor, self.params.MAX_TORQUE_FACTOR)

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
             self.takeover_req = 2
          self._reset_lat_state()
        else:
          if not CS.eps_active:
            if self.eps_active_last and CS.speed_kph <= LKAS_LIMITS.DISABLE_SPEED:
              self.takeover_req = 1
            self._activate_eps(CS, actuators.curvature)

          else:
            # first time it enters in the lateral active state, store the frame to check the rearm period
            if self.eps_activation_frame > 0 and (self.frame > self.eps_activation_frame + self.eps_rearm_frames):
              self._deactivate_eps()
            elif self.deactivation_in_progress:
              self._deactivate_eps()
            else:
              ##########
              ### START EPS ACTIVE
              ######
              # EPS is active, proceed with lateral control
              if self.eps_activation_frame == 0:
                self.eps_activation_frame = self.frame
              self.takeover_req = 0
              self.activation_request_frame = 0
              self.status = 4 # 4: EPS ACTIVE
              self.takeover_req_already_sent = False

              if (CS.out.steeringPressed):
                #### DRIVER STEERING DETECTED
                # If the driver is applying torque, give up the assist torque to avoid fighting the driver.
                self.apply_torque_factor = 0
                apply_new_torque_scaled = 0
                # apply_new_torque = 0
              else:
                # --- Requested torque (raw, still float) ------------------------------
                # actuators.torque is the model output in -1..1; scale to counts (x STEER_MAX).
                # Kept as a float here: it feeds the torque-factor curve below.
                #   ex: 0.25 * 250 = 62.5
                actuatorsRequestedTorque = CC.actuators.torque * self.params.STEER_MAX

                # --- Torque factor (dynamic EPS gain, MIN..MAX) -----------------------
                # The EPS multiplies our command by factor/100. Small requests get a low
                # factor (gentle), big ones a high factor, via a slightly convex curve.
                # ratio = normalized request magnitude, clamped 0..1, raised to 1.2.
                #   ex: (62.5/250) ** 1.2 = 0.25 ** 1.2 = 0.19
                ratio = min(1.0, (abs(actuatorsRequestedTorque) / float(self.params.STEER_MAX)) * 1.0) **1.2

                # Lerp ratio onto [MIN_TORQUE_FACTOR, MAX_TORQUE_FACTOR], then clamp.
                #   ex: 15 + 0.19 * (100 - 15) = 31
                self.apply_torque_factor = int(self.params.MIN_TORQUE_FACTOR + ratio * (self.params.MAX_TORQUE_FACTOR - self.params.MIN_TORQUE_FACTOR))
                self.apply_torque_factor = max(self.params.MIN_TORQUE_FACTOR, min(self.apply_torque_factor, self.params.MAX_TORQUE_FACTOR))

                # --- Effective (scaled) torque ---------------------------------------
                # What the wheel actually gets = request * factor/100. Same "force at the
                # wheel" domain as the driver torque, so this is what the limiter compares.
                #   ex: round(62.5 * 31/100) = 19
                new_torque_scaled = int(round(actuatorsRequestedTorque * self.apply_torque_factor / 100))

                # --- Driver-aware rate/override limiter -------------------------------
                # Feed the EFFECTIVE (scaled) command: it is already in the driver-torque
                # domain, so the driver signal needs no conversion. The limiter rate-limits
                # the ramp (STEER_DELTA_UP/DOWN) and backs off when the driver pushes.
                # It always returns an int (see lateral.py).
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
        can_sends.append(create_lka_steering(self.packer, CC.latActive, can_torque, self.apply_torque_factor, self.status,apply_new_torque_scaled))
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

    if self.car_fingerprint in (CAR.PSA_PEUGEOT_3008,CAR.PSA_CITROEN_C4_SPACETOURER):
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
        self.driver_torque_counter += 1
        if self.driver_torque_counter >= self.next_driver_torque:
          msg = CS.steering
          counter = (msg['COUNTER'] + 1) % 16
          can_sends.append(create_driver_torque(self.packer, CS.steering, counter))
          self.driver_torque_counter = 0
          self.next_driver_torque = random.randint(500, 800)

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
      if self.takeover_req > 0 and self.frame % 2 == 0: # 50 Hz
        # if self.takeover_start_msg_frame == 0:
        #   self.takeover_start_msg_frame = self.frame
        can_sends.append(create_request_takeover(self.packer, CS.HS2_DYN_MDD_ETAT_2F6,self.takeover_req))
        # carlog.error("PSA_DEBUG sending to CAN create_request_takeover")
        # if self.frame > self.takeover_start_msg_frame + self.takeover_msg_duration: # 1 s
      # carlog.error("PSA_DEBUG takeover_req = False")
        self.takeover_req = 0
        # self.takeover_start_msg_frame = 0

    # speed_target_ms = None
    # if self.CP.openpilotLongitudinalControl:
    #   speed_target_ms = CC.hudControl.setSpeed
    # elif not self.CP_SP.pcmCruiseSpeed:
    #   icbm = CC_SP.intelligentCruiseButtonManagement
    #   if icbm.state != ICBMState.inactive:
    #     # ICBM vTarget is published in SI units so this works in both metric and
    #     # imperial UI modes. It includes dynamic map/vision turn speed targets.
    #     speed_target_ms = icbm.vTarget

    # if (speed_target_ms is not None
    #     and self.frame % self.params.STEER_STEP == 0
    #     and CC.enabled
    #     and CC.hudControl.speedVisible
    #     and CS.out.cruiseState.enabled
    #     and CS.out.vEgo > 1.0):
    #   set_speed_kph = psa_speed_setpoint_from_cluster_kph(speed_target_ms * CV.MS_TO_KPH)
    #   if 0 < set_speed_kph < 255:
    #     can_sends.append(
    #       set_speed(self.packer, CS.hs2_dat_mdd_cmd_452, set_speed_kph)
    #     )

    # if self.CP.openpilotLongitudinalControl and CC.enabled:
    #   # Disable ARTIV only for full openpilot longitudinal. ICBM deliberately
    #   # leaves ARTIV active: the stock controller executes our dynamic setpoint.
    #   if self.radar_disabled == 0:
    #     can_sends.append(create_disable_radar())
    #     self.radar_disabled = 1

    #   if self.frame % 100 == 0 and self.frame > 0:
    #     can_sends.append(make_tester_present_msg(0x6b6, 1, suppress_response=False))

    # Intelligent Cruise Button Management
    if self.frame % self.params.STEER_STEP == 0:
      can_sends.extend(
        IntelligentCruiseButtonManagementInterface.update(
          self, CC, CC_SP, CS, self.packer
        )
      )
    # Actuators output
    new_actuators = actuators.as_builder()
    if self.CP.steerControlType == SteerControlType.torque:
      # Keep last applied torque between 20 Hz LKA updates.
      # The EPS maintains assist longer than 50 ms, preventing gaps in actuator output.
      new_actuators.torque = self.apply_torque_scaled_last / self.params.STEER_MAX
      new_actuators.torqueOutputCan = self.apply_torque_scaled_last
      new_actuators.steeringAngleDeg = float(self.apply_can_torque_last)
      # new_actuators.curvature = temp_driverSteeringTorque   # lo vedi in juggle come carControl.actuatorsOutput.curvature
      # new_actuators.steeringAngleDeg = float(self.apply_torque_factor)

      # if self.frame % 100 == 0:
      #   carlog.error(f"PSA_DEBUG torque={new_actuators.torque:.3f} torque_can={self.apply_torque_scaled_last}")
    if self.frame % self.params.STEER_STEP == 0:
      self.latActiveLast = CC.latActive
      self.eps_active_last = CS.eps_active

    self.frame += 1
    return new_actuators, can_sends
