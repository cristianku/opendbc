from opendbc.can.packer import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.psa.psacan import create_lka_steering,  create_driver_torque, create_steering_hold, create_request_takeover
from opendbc.car.psa.values import CarControllerParams, CAR
from opendbc.car.psa.driver_torque_generator import DriverTorqueGenerator

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
    self.lat_activation_frame  = 0
    self.car_fingerprint = CP.carFingerprint
    self.params = CarControllerParams(CP)
    self.lka_relay_ready = False
    self.lka_relay_wait_frames = 100  # Wait 1 second (100 frames @ 100Hz)

    # Driver torque generator with configurable parameters
    self.driver_torque_gen = DriverTorqueGenerator(
        duration_range=(600, 800),
        std_divisor=4,
        peak_torque_range=(12, 18),
        opposite_probability=0.2,
        rate_hz=100,        # ← AGGIUNGI
        eps_max=20          # ← AGGIUNGI
    )
    self._last_driver_torque = 0  # last torque value sent, used for IS_DAT_DIRA updates
    self.dt_active = False
    self.dt_step = 0
    self.DT_PERIOD_FRAMES = 1500  # intervallo tra burst: 15s @100Hz

    # Logica di persistenza per STEERWHL_HOLD_BY_DRV
    self.hold_by_drv_active = False
    self.hold_by_drv_frames_remaining = 0
    self.HOLD_PERSISTENCE_FRAMES = 50  # Mantiene HOLD attivo per 0.5s


  def _adjust_torque_params(self, speed, steering_angle):
      """
      Dynamically adjust driver torque generation parameters based on driving conditions.

      Args:
          speed: vehicle speed in m/s
          steering_angle: current steering angle in degrees

      Returns:
          dict with updated parameters for the torque generator
      """
      # Default values
      duration_range = (600, 800)
      std_divisor = 4
      peak_torque_range = (12, 18)
      opposite_probability = 0.2

      # Adjust based on speed
      if speed > 25:  # >90 km/h
          duration_range = (800, 1000)
          std_divisor = 5  # più stretta e precisa
          peak_torque_range = (10, 15)
      elif speed > 15:  # >54 km/h
          duration_range = (600, 800)
          std_divisor = 4
          peak_torque_range = (12, 18)
      else:  # Low speed
          duration_range = (400, 600)
          std_divisor = 3.5  # più larga
          peak_torque_range = (15, 20)

      # Adjust based on steering angle
      if abs(steering_angle) > 45:
          duration_range = tuple(int(d * 0.7) for d in duration_range)
          peak_torque_range = tuple(int(p * 0.8) for p in peak_torque_range)
          opposite_probability = 0.1  # Meno variabilità in curva

      return {
          'duration_range': duration_range,
          'std_divisor': std_divisor,
          'peak_torque_range': peak_torque_range,
          'opposite_probability': opposite_probability
      }

  def _reset_lat_state(self):
    """Reset lateral control state."""
    self.status = 2
    self.apply_torque_factor = 0
    self.lat_activation_frame = 0

  def _set_lat_state_active(self):
    """Set EPS state as active."""
    self.status = 4
    self.lat_activation_frame = 0

  def _activate_eps(self, eps_active):
    """
    Handle EPS activation sequence and takeover request.
    STATUS transitions: 2 → 3 → 4 (READY → AUTHORIZED → ACTIVE)
    """

    # Save frame number when EPS first activates or re-activates
    if self.lat_activation_frame == 0:
      self.lat_activation_frame = self.frame
      self.takeover_req_sent = False


    if not eps_active: # and not CS.out.steeringPressed:
      # Issue takeover request if EPS is unavailable (e.g., speed < 50 km/h)
      if self.frame % 2 == 0: # 50 Hz
        if not self.takeover_req_sent:
          if (self.frame - self.lat_activation_frame ) > 10:
          # can_sends.append(create_request_takeover(self.packer, CS.HS2_DYN_MDD_ETAT_2F6,1))
            self.takeover_req_sent = True

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
    # Wait for relay to be ready before sending LKA messages
    if not self.lka_relay_ready:
      if self.frame > self.lka_relay_wait_frames:
        self.lka_relay_ready = True
      else:
        # Don't send LANE_KEEP_ASSIST yet - relay not ready
        new_actuators = actuators.as_builder()
        self.frame += 1
        return new_actuators, can_sends


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

            # Linear torque factor interpolation
            ratio = min(1.0, (abs(apply_new_torque) / float(self.params.STEER_MAX)) * 1.0)

            self.apply_torque_factor = int(self.params.MIN_TORQUE_FACTOR + ratio * (self.params.MAX_TORQUE_FACTOR - self.params.MIN_TORQUE_FACTOR))
            self.apply_torque_factor = max(self.params.MIN_TORQUE_FACTOR, min(self.apply_torque_factor, self.params.MAX_TORQUE_FACTOR))


        #
        # Send LKA steering message (every 5 frames)
        can_sends.append(create_lka_steering(self.packer, CC.latActive, apply_new_torque, self.apply_torque_factor, self.status))
        self.apply_torque_last = apply_new_torque

    # --- Driver torque generation (simulated torque input) ---
    if self.car_fingerprint in (CAR.PSA_PEUGEOT_3008,) and CC.latActive:

      # 1) Every 15s start a new Gaussian curve with dynamic parameters
      if (self.frame % self.DT_PERIOD_FRAMES) == 0:
        # Adjust parameters based on current driving conditions
        params = self._adjust_torque_params(CS.out.vEgo, CS.out.steeringAngleDeg)
        self.driver_torque_gen.reset(**params)
        self.dt_active = True
        self.dt_step = 0

      # 2) During active burst: send DRIVER_TORQUE each frame (100 Hz)
      if self.dt_active:
        driver_torque_float = self.driver_torque_gen.next_value()
        # Cast a int solo qui per il CAN message
        driver_torque = int(round(driver_torque_float))

        can_sends.append(create_driver_torque(self.packer, CS.steering, driver_torque))
        self._last_driver_torque = driver_torque

        # Gestione HOLD_BY_DRV con persistenza
        if abs(driver_torque) > 8:  # Soglia per attivare HOLD
          self.hold_by_drv_active = True
          self.hold_by_drv_frames_remaining = self.HOLD_PERSISTENCE_FRAMES
        elif self.hold_by_drv_frames_remaining > 0:
          self.hold_by_drv_frames_remaining -= 1
          if self.hold_by_drv_frames_remaining == 0:
            self.hold_by_drv_active = False
        else:
          self.hold_by_drv_active = False

        self.dt_step += 1

        # Fine del burst quando la sequenza è completa
        if self.driver_torque_gen.is_done():
          self.dt_active = False
          self.hold_by_drv_active = False

      # 3) Send IS_DAT_DIRA every frame (100 Hz) while burst is active OR hold persists
      if self.dt_active or self.hold_by_drv_active:
          can_sends.append(create_steering_hold(self.packer,
                                                CS.is_dat_dira,
                                                self._last_driver_torque,
                                                self.hold_by_drv_active))


    # --- Actuator outputs ---
    new_actuators = actuators.as_builder()
    if self.CP.steerControlType == SteerControlType.torque:
      # Maintain last torque output between 20 Hz LKA updates.
      # EPS holds torque >50 ms, preventing output gaps.
      new_actuators.torque = self.apply_torque_last / self.params.STEER_MAX
      new_actuators.torqueOutputCan = self.apply_torque_last

    self.frame += 1
    return new_actuators, can_sends