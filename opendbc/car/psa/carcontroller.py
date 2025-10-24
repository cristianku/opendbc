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
    self.apply_torque_factor = 0
    self.apply_torque = 0
    self.status = 2

  # def update(self, CC, CS, now_nanos):
  #   can_sends = []

    ### cristian
    # #############
    # # lateral control
    # ######
    # if self.frame % CarControllerParams.STEER_STEP == 0:
    #   if not CC.latActive:
    #     self.apply_torque = 0
    #     self.apply_torque_factor = 0
    #     self.status = 2

    #   else:

    #     if not CS.eps_active:
    #       # Openpilot is not activated
    #       if self.status == 4:
    #         self.status = 2
    #       else:
    #         self.status += 1
    #         if self.status > 4:
    #           self.status = 4
    #       # Progressive activation of the Torque Factor
    #       self.apply_torque_factor += 5
    #       if self.apply_torque_factor > 100:
    #           self.apply_torque_factor = 100

    #     else:
    #       self.apply_torque_factor = 100
    #       self.status = 4

    #       # Torque
    #       temp_new_torque = int(round(CC.actuators.torque * max(1 , CarControllerParams.STEER_MAX) ))

    #       self.apply_torque = apply_driver_steer_torque_limits(temp_new_torque, self.apply_torque_last,
    #                                                       CS.out.steeringTorque, CarControllerParams )


    #   # Message sent every CarControllerParams.STEER_STEP frames
    #   can_sends.append(create_lka_steering(self.packer,self.apply_torque,self.apply_torque_factor,self.status))
    #   # last sent value
    #   self.apply_torque_last = self.apply_torque
    ### end cristian


    # new_actuators = CC.actuators.as_builder()
    # new_actuators.torque = self.apply_torque / CarControllerParams.STEER_MAX
    # new_actuators.torqueOutputCan = self.apply_torque

    # # self.frame += 1
    # self.frame = (self.frame + 1) % 10000

    # return new_actuators, can_sends



  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators

    # lateral control
    if self.frame % 5 == 0:
      new_torque = int(round(CC.actuators.torque * CarControllerParams.STEER_MAX))
      self.apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                      CS.out.steeringTorque, CarControllerParams, CarControllerParams.STEER_MAX)

      # EPS disengages on steering override, activation sequence 2->3->4 to re-engage
      # STATUS  -  0: UNAVAILABLE, 1: UNSELECTED, 2: READY, 3: AUTHORIZED, 4: ACTIVE
      if not CC.latActive:
        self.status = 2
        self.apply_torque_factor = 0

      elif not CS.eps_active and not CS.out.steeringPressed:
        self.status = 2 if self.status == 4 else self.status + 1
        self.apply_torque_factor += 5
        if self.apply_torque_factor > 100:
          self.apply_torque_factor = 100
      else:
        self.status = 4
        self.apply_torque_factor = 100

      can_sends.append(create_lka_steering(self.packer, CC.latActive, self.apply_torque, self.status))

      self.apply_torque_last = self.apply_torque

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque / CarControllerParams.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque

    self.frame += 1
    return new_actuators, can_sends