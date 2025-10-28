from opendbc.can.packer import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.psa.psacan import create_lka_steering,  create_driver_torque, create_steering_hold, create_request_takeover
from opendbc.car.psa.psacan import create_fake_driver_torque
from opendbc.car.psa.values import CarControllerParams, CAR
from opendbc.car.psa.driver_torque_generator import DriverTorqueGenerator
import random
import math
import numpy as np

SteerControlType = structs.CarParams.SteerControlType

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.apply_torque_last = 0
    self.apply_torque_factor = 0
    self.apply_torque = 0
    self.status = 2
    self.takeover_req_sent = False
    # this is the frame when the latactive is being pressed
    self.lat_activation_frame  = 0
    self.car_fingerprint = CP.carFingerprint
    self.params = CarControllerParams(CP)
    self.driver_torque_gen = DriverTorqueGenerator()
    self.dt_active = False
    self.dt_step = 0
    self.DT_PERIOD_FRAMES = 500   # 5 s @ 100 Hz
    self.DT_BURST_LEN = 200       # 200 frame (Gaussian Curve ~2 s)
    self._last_driver_torque = 0  # for IS_DAT_DIRA

  def _reset_lat_state(self):
    self.status = 2
    self.apply_torque_factor = 0
    self.lat_activation_frame = 0

  def _set_lat_state_active(self):
    self.status = 4
    self.lat_activation_frame = 0

  def _activate_eps(self, eps_active):
    # Save the frame number when the LKA (steering assist) button is first pressed on the car
    if self.lat_activation_frame == 0:
      # first frame the EPS activate or re activate is sent
      self.lat_activation_frame = self.frame
      self.takeover_req_sent = False


    if not eps_active: # and not CS.out.steeringPressed:
      #######
      # Alarm - Takeover request!
      # EPS works from 50km/h - Takeover Request if speed is slower than 50
      ######
      if self.frame % 2 == 0: # 50 Hz
        if not self.takeover_req_sent:
          if (self.frame - self.lat_activation_frame ) > 10:
          # can_sends.append(create_request_takeover(self.packer, CS.HS2_DYN_MDD_ETAT_2F6,1))
            self.takeover_req_sent = True

      ######
      # EPS activation sequence 2->3->4 to re-engage
      # STATUS  -  0: UNAVAILABLE, 1: UNSELECTED, 2: READY, 3: AUTHORIZED, 4: ACTIVE
      ######
      self.status = 2 if self.status == 4 else self.status + 1
      # EPS likes a progressive activation of the Torque Factor
      self.apply_torque_factor += 10
      self.apply_torque_factor = min( self.apply_torque_factor, self.params.MAX_TORQUE_FACTOR)

  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators
    self.apply_new_torque = 0
    apply_new_torque = 0

    # lateral control
    if self.CP.steerControlType == SteerControlType.torque:
      if self.frame % self.params.STEER_STEP == 0:
        if not CC.latActive:
          self._reset_lat_state()
          self.dt_active = False
          self.dt_step = 0

        else:
          if not CS.eps_active: # and not CS.out.steeringPressed:
            self._activate_eps( CS.eps_active)

          else:
            ### START EPS ACTIVE
            self._set_lat_state_active()

            ######
            # TORQUE CALCULATION
            temp_torque = int(round(CC.actuators.torque * self.params.STEER_MAX))
            apply_new_torque = apply_driver_steer_torque_limits(temp_torque, self.apply_torque_last,
                                                            CS.out.steeringTorque, self.params, self.params.STEER_MAX)

            # Linearly increase torque factor
            ratio = min(1.0, (abs(apply_new_torque) / float(self.params.STEER_MAX)) * 1.0)

            self.apply_torque_factor = int(self.params.MIN_TORQUE_FACTOR + ratio * (self.params.MAX_TORQUE_FACTOR - self.params.MIN_TORQUE_FACTOR))
            self.apply_torque_factor = max(self.params.MIN_TORQUE_FACTOR, min(self.apply_torque_factor, self.params.MAX_TORQUE_FACTOR))


        #
        #####
        # CAN MESSAGE needs to be sent every 5 frames
        #  - psa.h  check_relay is set for PSA_LANE_KEEP_ASSIST
        ####
        can_sends.append(create_lka_steering(self.packer, CC.latActive, apply_new_torque, self.apply_torque_factor, self.status))
        # last sent value to the EPS
        self.apply_torque_last = apply_new_torque
        ### END EPS ACTIVE
        ##########


    # update Driver Torque

      if self.car_fingerprint in (CAR.PSA_PEUGEOT_3008,) and CC.latActive:

        # 1) every  5s start the Gaussian of 200 frames
        if (self.frame % self.DT_PERIOD_FRAMES) == 0:
          self.driver_torque_gen.reset()   # ← restart from the begin of Gaussian Curve
          self.dt_active = True
          self.dt_step = 0

        # 2) during the Gaussian send DRIVER_TORQUE at 100 Hz (un Guassian point per )
        if self.dt_active:
          driver_torque = self.driver_torque_gen.next_value()
          can_sends.append(create_driver_torque(self.packer, CS.steering, driver_torque))
          self._last_driver_torque = driver_torque

          self.dt_step += 1
          if self.dt_step >= self.DT_BURST_LEN:
            self.dt_active = False

        # 3) IS_DAT_DIRA at 10 Hz
        if (self.frame % 10) == 0 and self.dt_active:
          can_sends.append(create_steering_hold(self.packer, CS.is_dat_dira, self._last_driver_torque))


    # Actuators output
    new_actuators = actuators.as_builder()
    if self.CP.steerControlType == SteerControlType.torque:
      # Keep last applied torque between 20 Hz LKA updates.
      # The EPS maintains assist longer than 50 ms, preventing gaps in actuator output.
      new_actuators.torque = self.apply_torque_last / self.params.STEER_MAX
      new_actuators.torqueOutputCan = self.apply_torque_last

    self.frame += 1
    return new_actuators, can_sends