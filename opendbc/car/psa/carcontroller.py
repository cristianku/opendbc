from opendbc.can.packer import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.psa.psacan import create_lka_steering, create_driver_torque, create_steering_hold, create_request_takeover
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

  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators

    # lateral control
    if self.CP.steerControlType == SteerControlType.torque:
      if self.frame % 5 == 0:
        apply_new_torque = 0
        # EPS disengages on steering override, activation sequence 2->3->4 to re-engage
        # STATUS  -  0: UNAVAILABLE, 1: UNSELECTED, 2: READY, 3: AUTHORIZED, 4: ACTIVE
        if not CC.latActive:
          self.status = 2
          self.apply_torque_factor = 0
          self.takeover_req_sent = False

        elif not CS.eps_active: # and not CS.out.steeringPressed:
          # eps can become inactive under 54km/h
          # we need to follow the activation sequence

          if not takeover_req_sent and self.frame % 2 == 0: # 50 Hz
            can_sends.append(create_request_takeover(self.packer, CS.HS2_DYN_MDD_ETAT_2F6,1))
            self.takeover_req_sent = True
          self.status = 2 if self.status == 4 else self.status + 1

          #need to set to zero because the steering wheel is free or force
          # otherwise we would sent to the controller a wrong value, a value before the disengaging
          # if self.apply_torque_factor < CarControllerParams.MAX_TORQUE_FACTOR:
          #   self.apply_torque_factor += CarControllerParams.MAX_TORQUE_FACTOR
          # else:
          #   self.apply_torque_factor += 10

          # if self.apply_torque_factor >  CarControllerParams.MAX_TORQUE_FACTOR:
          #   self.apply_torque_factor =  CarControllerParams.MAX_TORQUE_FACTOR
          self.apply_torque_factor = 0
          # if self.apply_torque_factor < CarControllerParams.MAX_TORQUE_FACTOR:
          #   self.apply_torque_factor += 10
          # self.apply_torque_factor = min(self.apply_torque_factor, CarControllerParams.MAX_TORQUE_FACTOR)


        else:
          # EPS become active. THe first time we enter here the self.apply_torque_last is 0 either because its the first activation
          # or because a disengaging has happened( example speed drop below 54 km/h)
          self.takeover_req_sent = False
          self.status = 4

          temp_torque = int(round(CC.actuators.torque * CarControllerParams.STEER_MAX))
          apply_new_torque = apply_driver_steer_torque_limits(temp_torque, self.apply_torque_last,
                                                          CS.out.steeringTorque, CarControllerParams, CarControllerParams.STEER_MAX)

          # self.apply_torque_factor = CarControllerParams.MAX_TORQUE_FACTOR
          # Linearly increase torque factor near STEER_MAX to gain higher resolution (more steps) at high torque.
          # Higher torque -> lower torque factor (less resolution needed)
          ratio = min(1.0, abs(self.apply_torque) / float(CarControllerParams.STEER_MAX))
          target_tf = int(
              CarControllerParams.MAX_TORQUE_FACTOR
              - ratio * (CarControllerParams.MAX_TORQUE_FACTOR - CarControllerParams.MIN_TORQUE_FACTOR)
          )
          self.apply_torque_factor = max(CarControllerParams.MIN_TORQUE_FACTOR, target_tf)

          # self.apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
          #                                                 0, CarControllerParams, CarControllerParams.STEER_MAX)

        # emulate driver torque message at 1 Hz
        # if self.frame % 100 == 0:
        #   can_sends.append(create_driver_torque(self.packer, CS.steering))

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