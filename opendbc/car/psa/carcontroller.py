from opendbc.can.packer import CANPacker
from opendbc.car import Bus, structs, make_tester_present_msg
from opendbc.car.carlog import carlog
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
# from opendbc.car.psa.psacan import create_lka_steering, create_driver_torque, create_steering_hold, create_resume_acc, create_disable_radar, create_HS2_DYN1_MDD_ETAT_2B6, create_HS2_DYN_MDD_ETAT_2F6
from opendbc.car.psa.psacan import create_lka_steering, create_driver_torque, create_steering_hold, create_resume_acc,  create_HS2_DYN1_MDD_ETAT_2B6, create_HS2_DYN_MDD_ETAT_2F6
from opendbc.car.psa.values import CarControllerParams, CAR
from cereal import messaging
from numpy import interp

import random
import math

SteerControlType = structs.CarParams.SteerControlType
sm = messaging.SubMaster(['modelV2'], poll='modelV2')

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.apply_torque_last = 0
    self.apply_can_torque_last = 0  # raw CAN torque logged to steeringAngleDeg (debug); init so it always exists
    self.apply_torque_factor = 0
    self.apply_torque = 0
    self.status = 2
    self.takeover_req_sent = False
    # this is the frame when the latactive is being pressed
    self.lat_activation_frame  = 0
    self.lat_active_last = False
    self.car_fingerprint = CP.carFingerprint
    self.params = CarControllerParams(CP)
    self.radar_disabled = 0
    self.bars = 4
    self.steering_hold_counter = 0
    self.next_steering_hold = random.randint(8, 12)  # ~10Hz con jitter ±20%
    self.driver_torque_counter = 0
    self.next_driver_torque = random.randint(500, 800)  # 5–8 s @100 Hz

  def _reset_lat_state(self):
    self.status = 2
    self.apply_torque_factor = 0
    self.takeover_req_sent = False
    self.lat_activation_frame = 0

  def _activate_eps(self, eps_active):
    # Save the frame number when the LKA (steering assist) button is first pressed on the car
    if self.lat_activation_frame == 0:
      # first frame the EPS activate or re activate is sent
      self.lat_activation_frame = self.frame
      # self.takeover_req_sent = False


    if not eps_active: # and not CS.out.steeringPressed:
      #######
      # Alarm - Takeover request!
      # EPS works from 50km/h - Takeover Request if speed is slower than 50
      ######
      # if not self.takeover_req_sent and self.frame % 2 == 0: # 50 Hz
      #   if (self.frame - self.lat_activation_frame) > 10:
        # can_sends.append(create_request_takeover(self.packer, CS.HS2_DYN_MDD_ETAT_2F6,1))
          # self.takeover_req_sent = True

      ######
      # EPS activation sequence 2->3->4 to re-engage
      # STATUS  -  0: UNAVAILABLE, 1: UNSELECTED, 2: READY, 3: AUTHORIZED, 4: ACTIVE
      ######
      self.status = 2 if self.status == 4 else self.status + 1
      # EPS likes a progressive activation of the Torque Factor
      self.apply_torque_factor += 10
      self.apply_torque_factor = min( self.apply_torque_factor, self.params.MAX_TORQUE_FACTOR)

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators
    self.apply_new_torque = 0
    # apply_new_torque = 0
    temp_driverSteeringTorque = 0
    new_torque_scaled = 0
    apply_new_torque_scaled = 0
    if CC.latActive != self.lat_active_last:
      carlog.error(f"PSA_DEBUG latActive={CC.latActive}")
      self.lat_active_last = CC.latActive

    # lateral control
    if self.CP.steerControlType == SteerControlType.torque:
      if self.frame % self.params.STEER_STEP == 0:
        if not CC.latActive:
          self._reset_lat_state()
        else:
          if not CS.eps_active: # and not CS.out.steeringPressed:
            self._activate_eps( CS.eps_active)

          else:
            ##########
            ### START EPS ACTIVE
            ######
            # EPS is active, proceed with lateral control
            self.lat_activation_frame = 0
            self.status = 4 # 4: EPS ACTIVE

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
              apply_new_torque_scaled = apply_driver_steer_torque_limits(new_torque_scaled, self.apply_torque_last,
                                                              temp_driverSteeringTorque, self.params, self.params.STEER_MAX)

        # --- Back to a raw CAN command ----------------------------------------
        # The EPS re-applies factor/100, so undo the scaling to recover the raw value
        # to send. can_torque and factor travel as two separate CAN signals. Sent
        # every STEER_STEP frames; psa.h check_relay is set for PSA_LANE_KEEP_ASSIST.
        #   ex: round(19 / 31 * 100) = 61  ->  EPS redoes 61 * 31/100 = 19 (effective)
        if self.apply_torque_factor > 0:
          can_torque = int(round(apply_new_torque_scaled / self.apply_torque_factor *100))
        else:
          can_torque = 0
        # can_sends.append(create_lka_steering(self.packer, CC.latActive, can_torque, self.apply_torque_factor, self.status))
        can_sends.append(create_lka_steering(self.packer, CC.latActive, can_torque, self.apply_torque_factor, self.status,apply_new_torque_scaled))
        # Remember the effective (scaled) value for the next frame's rate limit.
        self.apply_torque_last = apply_new_torque_scaled
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
    #     can_sends.append(create_HS2_DYN1_MDD_ETAT_2B6(self.packer, self.frame // 2, actuators.accel, CS.out.cruiseState.enabled, CS.out.gasPressed, braking, CS.out.brakePressed, CS.out.standstill, torque))
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



    if self.car_fingerprint in (CAR.PSA_PEUGEOT_3008,):
      if not CC.latActive:
        self.driver_torque_counter = 0
        self.next_driver_torque = random.randint(500, 800)
      else:
    #     # --- HOLD HANDS (~10 Hz con jitter 8–12 frame) ---
        self.steering_hold_counter += 1
        if self.steering_hold_counter >= self.next_steering_hold:
          can_sends.append(create_steering_hold(self.packer, CC.latActive, CS.is_dat_dira))
          self.steering_hold_counter = 0
          self.next_steering_hold = random.randint(8, 12)
    #     # --- DRIVER TORQUE (ogni 5–8 s) ---
        self.driver_torque_counter += 1
        if self.driver_torque_counter >= self.next_driver_torque:
          msg = CS.steering
          counter = (msg['COUNTER'] + 1) % 16
          can_sends.append(create_driver_torque(self.packer, CS.steering, counter))
          self.driver_torque_counter = 0
          self.next_driver_torque = random.randint(500, 800)

    # Actuators output
    new_actuators = actuators.as_builder()
    if self.CP.steerControlType == SteerControlType.torque:
      # Keep last applied torque between 20 Hz LKA updates.
      # The EPS maintains assist longer than 50 ms, preventing gaps in actuator output.
      new_actuators.torque = self.apply_torque_last / self.params.STEER_MAX
      new_actuators.torqueOutputCan = self.apply_torque_last
      new_actuators.steeringAngleDeg = float(self.apply_can_torque_last)
      # new_actuators.curvature = temp_driverSteeringTorque   # lo vedi in juggle come carControl.actuatorsOutput.curvature
      # new_actuators.steeringAngleDeg = float(self.apply_torque_factor)

      # if self.frame % 100 == 0:
      #   carlog.error(f"PSA_DEBUG torque={new_actuators.torque:.3f} torque_can={self.apply_torque_last}")

    self.frame += 1
    return new_actuators, can_sends