from opendbc.can.packer import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.psa.psacan import create_lka_steering, create_steering_hold, create_driver_torque, calculate_apply_torque, calculate_apply_factor, calculate_LKA_status
from opendbc.car.psa.values import CarControllerParams
import math

# def torque_to_factor(torque: int, steer_max: int) -> int:
#     """Converte torque (0–steer_max) in torque factor (0–100) con curva logaritmica."""
#     if steer_max <= 0:
#         return 0
#     t = max(0, min(torque, steer_max))
#     return int(round(100 * math.log(t + 1) / math.log(steer_max + 1)))

# def smooth_torque_factor(curr_torque: int, prev_torque: int, steer_max: int, alpha: float = 0.1) -> int:
#     """Applica smoothing esponenziale sul torque factor calcolato logaritmicamente."""
#     curr_factor = torque_to_factor(curr_torque, steer_max)
#     prev_factor = torque_to_factor(prev_torque, steer_max)
#     return int(round(alpha * curr_factor + (1 - alpha) * prev_factor))


# def status_cycle(status:int) -> int:
#   new_status = status + 1
#   if new_status == 4:
#     new_status = 2

#   return new_status


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
    actuators = CC.actuators

    #############
    # lateral control
    ######
    if CC.latActive:
        #  emulate driver torque message at 1 Hz
      if self.frame % 100 == 0:
        if CS.eps_active:
          can_sends.append(create_driver_torque(self.packer, CS.steering))

      if self.frame % 10 == 0:
        if CS.eps_active:
          # send steering wheel hold message at 10 Hz to keep EPS engaged
          can_sends.append(create_steering_hold(self.packer, CC.latActive, CS.is_dat_dira))

      if self.frame % CarControllerParams.STEER_STEP == 0:
        if CS.eps_active:
          # The torque will be calculated only if the eps is active, if its not active should be sent zero
          # It could happen that the EPS was engaged and then it disengage even with CC.Latactive True
          self.apply_torque = calculate_apply_torque(CC.actuators.torque, CS.out.steeringTorque,self.apply_torque_last, CarControllerParams)
            # trying to emulate the progressive activation of the EPS, like in the stock LKA
          self.apply_torque_factor = calculate_apply_factor(self.apply_torque_factor)

          self.status = calculate_LKA_status(CC.latActive, CS.eps_active, self.status)

        else:
          self.apply_torque = 0
          self.apply_torque_factor = 0
          # Trying to activate the EPS with 2(ready), 3 (authorized), 4(active)
          self.status = calculate_LKA_status(CC.latActive, CS.eps_active, self.status)


        # LKA can message sent every CarControllerParams.STEER_STEP frames
        can_sends.append(create_lka_steering(self.packer,self.apply_torque,self.apply_torque_factor,self.status))
        # last sent value
        self.apply_torque_last = self.apply_torque
    else:
      # OpenPilot is not active  → cleaning the variables
      self.apply_torque = 0
      self.apply_torque_last = 0
      self.apply_torque_factor = 0
      self.status = self.READY

    # The apply torque is calculated every 5 frames ( depending on CarControllerParams.STEER_STEP )
    # The information for the actuators is sent every frame. It means that we need to sent the last known value
    # that was sent to the LKA even if its not on the current frame
    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque / CarControllerParams.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque

    # self.frame += 1
    self.frame = (self.frame + 1) % 10000

    return new_actuators, can_sends
