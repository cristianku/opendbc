from opendbc.can.packer import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.psa.psacan import create_lka_steering, create_driver_torque, create_steering_hold, create_request_takeover, get_apply_torque, get_torque_factor
from opendbc.car.psa.values import CarControllerParams
import math

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

  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators
    apply_new_torque = 0

    # lateral control
    if self.CP.steerControlType == SteerControlType.torque:

      if not CC.latActive:
        self.status = 2
        self.apply_torque_factor = 0
        self.takeover_req_sent = False
        self.lat_activation_frame = 0

      else:
        # Save the frame number when the LKA (steering assist) button is first pressed on the car
        if self.lat_activation_frame == 0:
          self.lat_activation_frame = self.frame

        if self.frame % CarControllerParams.STEER_STEP == 0:

          if not CS.eps_active: # and not CS.out.steeringPressed:
            #######
            # Alarm - Takeover request
            # EPS works from 50km/h - Takeover Request if speed is slower than 50
            ######
            if not self.takeover_req_sent and self.frame % 2 == 0: # 50 Hz
              if (self.frame - self.lat_activation_frame) > 10:
              # can_sends.append(create_request_takeover(self.packer, CS.HS2_DYN_MDD_ETAT_2F6,1))
                self.takeover_req_sent = True

            ######
            # EPS activation sequence 2->3->4 to re-engage
            # STATUS  -  0: UNAVAILABLE, 1: UNSELECTED, 2: READY, 3: AUTHORIZED, 4: ACTIVE
            ######
            self.status = 2 if self.status == 4 else self.status + 1
            # EPS likes a progressive activation of the Torque Factor
            self.apply_torque_factor += 10
            self.apply_torque_factor = min( self.apply_torque_factor, CarControllerParams.MAX_TORQUE_FACTOR)

          else:
            ######
            # EPS activate
            ######
            self.takeover_req_sent = False
            self.status = 4 # 4: EPS ACTIVE

            ######
            # TORQUE CALCULATION
            #####
            # Torque calculation and normalizaiton
            # temp_torque = int(round(CC.actuators.torque * CarControllerParams.STEER_MAX))
            # apply_new_torque = apply_driver_steer_torque_limits(temp_torque, self.apply_torque_last,
            #                                                 CS.out.steeringTorque, CarControllerParams, CarControllerParams.STEER_MAX)

            apply_new_torque = get_apply_torque(CC.actuators.torque , CS, CarControllerParams, self.apply_torque_last)
            # Linearly increase torque factor
            # ratio = min(1.0, abs(apply_new_torque) / float(CarControllerParams.STEER_MAX))
            # target_tf = int(
            #     CarControllerParams.MAX_TORQUE_FACTOR
            #     - ratio * (CarControllerParams.MAX_TORQUE_FACTOR - CarControllerParams.MIN_TORQUE_FACTOR)
            # )
            # self.apply_torque_factor = max(CarControllerParams.MIN_TORQUE_FACTOR, target_tf)

            self.apply_torque_factor = get_torque_factor(apply_new_torque, CarControllerParams)
          # emulate driver torque message at 1 Hz
          # if self.frame % 100 == 0:
          #   can_sends.append(create_driver_torque(self.packer, CS.steering))

        #####
        # CAN MESSAGE FOR LANE KEEP ASSIST SENT ALWAYS IF LATACTIVE
        ####
        can_sends.append(create_lka_steering(self.packer, CC.latActive, apply_new_torque, self.apply_torque_factor, self.status))

        self.apply_torque_last = apply_new_torque

    # if self.frame % 10 == 0:
    #   # send steering wheel hold message
    #   can_sends.append(create_steering_hold(self.packer, CC.latActive, CS.is_dat_dira))

    # Actuators output
    new_actuators = actuators.as_builder()
    if self.CP.steerControlType == SteerControlType.torque:
      # Keep last applied torque between 20 Hz LKA updates.
      # The EPS maintains assist longer than 50 ms, preventing gaps in actuator output.
      new_actuators.torque = self.apply_torque_last / CarControllerParams.STEER_MAX
      new_actuators.torqueOutputCan = self.apply_torque_last

    self.frame += 1
    return new_actuators, can_sends