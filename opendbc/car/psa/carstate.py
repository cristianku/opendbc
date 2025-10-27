from opendbc.car import structs, Bus
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.psa.values import CAR, DBC, CarControllerParams, LKAS_LIMITS
from opendbc.car.interfaces import CarStateBase
import copy
# from openpilot.common.filter_simple import FirstOrderFilter
# from opendbc.car import DT_CTRL
# from collections import deque


GearShifter = structs.CarState.GearShifter
TransmissionType = structs.CarParams.TransmissionType


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    # --- driver torque filtering state (Toyota-style) ---
    # self._drv_lp = FirstOrderFilter(0.0, DT_CTRL, 0.25)  # tau=0.25 s
    # self._drv_deadband = 0.3                             # Nm, snap-to-zero
    # self._drv_press_thr = 1.0                            # Nm, pressed threshold
    # self._drv_press_ms = 200                             # ms, debounce
    # self._drv_press_frames = max(1, int(self._drv_press_ms / (DT_CTRL * 1000)))
    # self._drv_press_cnt = 0

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.main]
    cp_adas = can_parsers[Bus.adas]
    cp_cam = can_parsers[Bus.cam]
    ret = structs.CarState()

    # car speed
    self.parse_wheel_speeds(ret,
      cp.vl['Dyn4_FRE']['P263_VehV_VPsvValWhlFrtL'],
      cp.vl['Dyn4_FRE']['P264_VehV_VPsvValWhlFrtR'],
      cp.vl['Dyn4_FRE']['P265_VehV_VPsvValWhlBckL'],
      cp.vl['Dyn4_FRE']['P266_VehV_VPsvValWhlBckR'],
    )
    ret.yawRate = cp_adas.vl['HS2_DYN_UCF_MDD_32D']['VITESSE_LACET_BRUTE'] * CV.DEG_TO_RAD
    ret.standstill = cp.vl['Dyn4_FRE']['P263_VehV_VPsvValWhlFrtL'] < 0.1

    # gas
    if self.CP.carFingerprint == CAR.PSA_PEUGEOT_3008:
      ret.gasPressed = cp.vl['Dyn5_CMM']['P334_ACCPed_Position'] > 0
    else:
      ret.gasPressed = cp_cam.vl['DRIVER']['GAS_PEDAL'] > 0

    # brake pressed
    ret.brakePressed = bool(cp_cam.vl['Dat_BSI']['P013_MainBrake'])

    # brake pressure
    if self.CP.carFingerprint == CAR.PSA_PEUGEOT_3008:
      raw = cp.vl["Dyn2_FRE"]["BRAKE_PRESSURE"]
      ret.brake = max(0.0, float(raw) - 550.0)  # clamp a 0

    # parking brake
    ret.parkingBrake = cp.vl['Dyn_EasyMove']['P337_Com_stPrkBrk'] == 1 # 0: disengaged, 1: engaged, 3: brake actuator moving

    # steering wheel
    STEERING_ALT_BUS = {
      CAR.PSA_PEUGEOT_208: cp.vl,
      CAR.PSA_PEUGEOT_508: cp_cam.vl,
      CAR.PSA_PEUGEOT_3008: cp.vl,
    }
    bus = STEERING_ALT_BUS[self.CP.carFingerprint]
    ret.steeringAngleDeg = bus['STEERING_ALT']['ANGLE'] # EPS
    if self.CP.carFingerprint == CAR.PSA_PEUGEOT_3008:
      # PSA EPS encodes the steering rotation direction bit inverted from the driver's perspective:
      #   RATE_SIGN = 0 → clockwise (right turn)
      #   RATE_SIGN = 1 → anticlockwise (left turn)
      # Invert the sign to match OpenPilot's convention: right = positive, left = negative.
      ret.steeringRateDeg = bus['STEERING_ALT']['RATE'] * (1 - 2 * bus['STEERING_ALT']['RATE_SIGN'])
    else:
      # Convert EPS direction bit [0,1] to signed multiplier [-1,+1]
      # Standard convention: 0 → left (negative), 1 → right (positive)
      ret.steeringRateDeg  = bus['STEERING_ALT']['RATE'] * (2 * bus['STEERING_ALT']['RATE_SIGN'] - 1)

    if self.CP.carFingerprint == CAR.PSA_PEUGEOT_3008:
      ret.genericToggle = (int(cp.vl["IS_DAT_DIRA"]["ETAT_DA_DYN"]) == 1) # 0 = Normal, 1 = Dynamic/Sport, 2 = Adjustable

      ret.steeringTorque  = cp.vl['IS_DAT_DIRA']['EPS_TORQUE'] * 10
      ret.steeringTorqueEps = 0.0
      # ret.steeringPressed = (self._drv_press_cnt >= self._drv_press_frames)

    else:
      ret.steeringTorque = cp.vl['STEERING']['DRIVER_TORQUE']
      ret.steeringTorqueEps = cp.vl['IS_DAT_DIRA']['EPS_TORQUE']

    if self.CP.carFingerprint == CAR.PSA_PEUGEOT_3008:
      # Peugeot 3008: EPS_TORQUE represents only driver-applied torque (no motor assist).
      # The signal is already smoothed by the EPS ECU, so update_steering_pressed is unnecessary.
      ret.steeringPressed = abs(ret.steeringTorque) > LKAS_LIMITS.STEER_THRESHOLD
    else:
      ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE, 5)

    self.eps_active = cp.vl['IS_DAT_DIRA']['EPS_STATE_LKA'] == 3 # 0: Unauthorized, 1: Authorized, 2: Available, 3: Active, 4: Defect
    self.is_dat_dira = copy.copy(cp.vl['IS_DAT_DIRA'])
    self.steering = copy.copy(cp.vl['STEERING'])
    self.HS2_DYN_MDD_ETAT_2F6 =copy.copy(cp_adas.vl['HS2_DYN_MDD_ETAT_2F6'])

    # cruise
    ret.cruiseState.speed = cp_adas.vl['HS2_DAT_MDD_CMD_452']['SPEED_SETPOINT'] * CV.KPH_TO_MS # set to 255 when ACC is off, -2 kph offset from dash speed
    ret.cruiseState.enabled = cp_adas.vl['HS2_DAT_MDD_CMD_452']['RVV_ACC_ACTIVATION_REQ'] == 1
    ret.cruiseState.available = True # not available for CC-only
    ret.cruiseState.nonAdaptive = False # not available for CC-only
    ret.cruiseState.standstill = False # not available for CC-only
    ret.accFaulted = False # not available for CC-only

    # gear
    if bool(cp_cam.vl['Dat_BSI']['P103_Com_bRevGear']):
      ret.gearShifter = GearShifter.reverse
    else:
      ret.gearShifter = GearShifter.drive

    # blinkers
    blinker = cp_cam.vl['HS2_DAT7_BSI_612']['CDE_CLG_ET_HDC']
    if self.CP.carFingerprint == CAR.PSA_PEUGEOT_3008:
      ret.leftBlinker = blinker == 2
      ret.rightBlinker = blinker == 1
    else:
      ret.leftBlinker = blinker == 1
      ret.rightBlinker = blinker == 2

    # Blind sensor ( there is not left and right )
    if self.CP.carFingerprint == CAR.PSA_PEUGEOT_3008:
      ret.leftBlindspot = cp_adas.vl["HS2_DYN_MDD_ETAT_2F6"]["BLIND_SENSOR"] != 0
      ret.rightBlindspot = cp_adas.vl["HS2_DYN_MDD_ETAT_2F6"]["BLIND_SENSOR"] != 0

    # Auto Braking in progress
    if self.CP.carFingerprint == CAR.PSA_PEUGEOT_3008:
      ret.stockAeb = cp_adas.vl["HS2_DYN1_MDD_ETAT_2B6"]["AUTO_BRAKING_STATUS"] == 1

    # lock info
    ret.doorOpen = any((cp_cam.vl['Dat_BSI']['DRIVER_DOOR'], cp_cam.vl['Dat_BSI']['PASSENGER_DOOR']))
    ret.seatbeltUnlatched = cp_cam.vl['RESTRAINTS']['DRIVER_SEATBELT'] != 2

    return ret

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.main: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
      Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 1),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
