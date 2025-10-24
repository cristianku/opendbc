from opendbc.can.packer import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.psa.psacan import create_lka_steering
from opendbc.car.psa.values import CarControllerParams
import math

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.apply_torque_last = 0
    self.torque_factor_smoothed = 0  # Initialize smoothed value as int
    self.smoothing_alpha = 0.1  # Smoothing factor (0 < alpha < 1, smaller = stronger smoothing)
    self.apply_torque_factor = 0
    self.apply_torque = 0
    self.status = 2
    # States
    self.READY  = 2
    self.AUTH   = 3
    self.ACTIVE = 4

  def update(self, CC, CS, now_nanos):
    can_sends = []

    #############
    # lateral control
    ######
    if self.frame % CarControllerParams.STEER_STEP == 0:
      if not CC.latActive:
        self.apply_torque = 0
        self.apply_torque_factor = 0
        self.status = 2

      else:
        if not CS.eps_active:
          # Openpilot is not activated
          if self.status == 4:
            self.status = 2
          else:
            self.status += 1
            if self.status > 4:
              self.status = 4

        else:
          self.status = 4

          # Progressive activation of the Torque Factor
          self.apply_torque_factor += 5

          if self.apply_torque_factor > 100:
              self.apply_torque_factor = 100

          # Torque
          temp_new_torque = int(round(CC.actuators.torque * max(1 , CarControllerParams.STEER_MAX) ))

          self.apply_torque = apply_driver_steer_torque_limits(temp_new_torque, self.apply_torque_last,
                                                          CS.out.steeringTorque, CarControllerParams )



      # Message sent every CarControllerParams.STEER_STEP frames
      can_sends.append(create_lka_steering(self.packer,self.apply_torque,self.apply_torque_factor,self.status))
      # last sent value
      self.apply_torque_last = self.apply_torque

    #  emulate driver torque message at 1 Hz
    # if self.frame % 100 == 0 and CS.eps_active:
    #     can_sends.append(create_driver_torque(self.packer, CS.steering))


    # if self.frame % 10 == 0 and CS.eps_active:
    #     # send steering wheel hold message at 10 Hz to keep EPS engaged
    #     can_sends.append(create_steering_hold(self.packer, CC.latActive, CS.is_dat_dira))

    # The apply torque is calculated every 5 frames ( depending on CarControllerParams.STEER_STEP )
    # The information for the actuators is sent every frame. It means that we need to sent the last known value
    # that was sent to the LKA even if its not on the current frame
    new_actuators = CC.actuators.as_builder()
    new_actuators.torque = self.apply_torque / CarControllerParams.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque

    # self.frame += 1
    self.frame = (self.frame + 1) % 10000

    return new_actuators, can_sends
