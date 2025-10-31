from opendbc.can.packer import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.psa.psacan import create_lka_steering,  create_driver_torque, create_steering_hold, create_request_takeover, relay_driver_torque, create_wheel_speed_spoof
from opendbc.car.psa.values import CarControllerParams, CAR
from opendbc.car.psa.driver_torque_generator import DriverTorqueGenerator
import random
VisualAlert = structs.CarControl.HUDControl.VisualAlert

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
    # frame counter when LKA (lateral control) becomes active
    self.car_fingerprint = CP.carFingerprint
    self.params = CarControllerParams(CP)
    self.eps_cycle_initial_frame = 0

    # Driver torque generator with configurable parameters
    self.driver_torque_gen = DriverTorqueGenerator()

  def _reset_lat_state(self):
    """Reset lateral control state."""
    self.status = 2
    self.apply_torque_factor = 0
    self.eps_cycle_initial_frame = 0

  def _set_lat_state_active(self):
    """Set EPS state as active."""
    self.status = 4
    self.eps_cycle_initial_frame = 0

  def _activate_eps(self, eps_active):
    """
    Handle EPS activation sequence and takeover request.
    STATUS transitions: 2 → 3 → 4 (READY → AUTHORIZED → ACTIVE)
    """
    if self.eps_cycle_initial_frame == 0:
      self.eps_cycle_initial_frame = self.frame
      self.apply_torque_factor = 0

    if abs( self.frame - self.eps_cycle_initial_frame) % 10 == 0:
      #  2 = Critical request
      self.steer_hud_alert = 2

    # EPS activation sequence 2→3→4
    self.status = 2 if self.status == 4 else self.status + 1

    # Gradual ramp-up of torque factor during reactivation
    self.apply_torque_factor += 10
    self.apply_torque_factor = min( self.apply_torque_factor, self.params.MAX_TORQUE_FACTOR)

  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators
    self.apply_new_torque = 0
    apply_new_torque = 0
    hud_control = CC.hudControl
    # HS2_DYN_MDD_ETAT_2F6
    if hud_control.visualAlert == VisualAlert.steerRequired:
      #  2 = Critical request
      self.steer_hud_alert = 2
    elif hud_control.visualAlert ==  VisualAlert.ldw:
      #  1 = Non Critical Request
      # if in the buffer there is a critical = 2 this has higher prio
      self.steer_hud_alert = max(self.steer_hud_alert, 1 )
    # since the message is sent only every 2 frames, it will be set to zero by the sender
    # else :
    #   self.steer_hud_alert = 0


    # --- Lateral control logic ---
    if self.CP.steerControlType == SteerControlType.torque:
      if self.frame % self.params.STEER_STEP == 0:
        if not CC.latActive:
          # Lateral control disabled: reset torque state and stop driver torque burst
          self._reset_lat_state()
          self.dt_active = False
          self.dt_step = 0

        else:
          if not CS.eps_active:

            self._activate_eps( CS.eps_active)

          else:
            # EPS ACTIVE — perform steering torque control
            self._set_lat_state_active()

            # --- Torque calculation ---
            temp_torque = int(round(CC.actuators.torque * self.params.STEER_MAX))
            apply_new_torque = apply_driver_steer_torque_limits(temp_torque, self.apply_torque_last,
                                                            CS.out.steeringTorque, self.params, self.params.STEER_MAX)
            # this is just to test the alert
            if apply_new_torque > 20:
                #  1 = Non Critical alert
                self.steer_hud_alert = 1

            # Linear torque factor interpolation
            ratio = min(1.0, (abs(apply_new_torque) / float(self.params.STEER_MAX)) ** 2)

            self.apply_torque_factor = int(self.params.MIN_TORQUE_FACTOR + ratio * (self.params.MAX_TORQUE_FACTOR - self.params.MIN_TORQUE_FACTOR))
            self.apply_torque_factor = max(self.params.MIN_TORQUE_FACTOR, min(self.apply_torque_factor, self.params.MAX_TORQUE_FACTOR))


        #
        # Send LKA steering message (every 5 frames)
        can_sends.append(create_lka_steering(self.packer, CC.latActive, apply_new_torque, self.apply_torque_factor, self.status))
        self.apply_torque_last = apply_new_torque

    if self.frame % 2 == 0:
      if self.steer_hud_alert:
        can_sends.append(create_request_takeover(self.packer, CS.HS2_DYN_MDD_ETAT_2F6,self.steer_hud_alert))
        self.steer_hud_alert = 0

    # if self.car_fingerprint in (CAR.PSA_PEUGEOT_3008,) and CC.latActive:
      # torque = self.driver_torque_gen.next_value()
      # can_sends.append(create_driver_torque(self.packer, CS.steering, torque ))
      # if self.frame % 1 == 0:
      #   can_sends.append(create_steering_hold(self.packer, CS.is_dat_dira, torque ))

      # --- Wheel speed spoofing @ 50Hz to keep EPS active (min 55 km/h) ---
      # if self.frame % 2 == 0:  # 50 Hz
      #   if CC.latActive:
      #     can_sends.append(create_wheel_speed_spoof(self.packer, CS.dyn4_fre, min_speed=55.0))


    # --- Actuator outputs ---
    new_actuators = actuators.as_builder()
    if self.CP.steerControlType == SteerControlType.torque:
      # Maintain last torque output between 20 Hz LKA updates.
      # EPS holds torque >50 ms, preventing output gaps.
      new_actuators.torque = self.apply_torque_last / self.params.STEER_MAX
      new_actuators.torqueOutputCan = self.apply_torque_last

    self.frame += 1
    return new_actuators, can_sends