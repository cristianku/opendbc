from opendbc.car import structs, Bus
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.psa.values import CAR, DBC, CarControllerParams
# , LKAS_LIMITS
from opendbc.car.interfaces import CarStateBase
import copy
import math
from opendbc.car.common.filter_simple import FirstOrderFilter  # NB: version inside opendbc (like Toyota), NOT openpilot.common
from opendbc.car import DT_CTRL
# from collections import deque


GearShifter = structs.CarState.GearShifter
TransmissionType = structs.CarParams.TransmissionType


class CarState(CarStateBase):
  # Low-pass filter for the raw (noisy) DRIVER_TORQUE signal.
  # The filter must live in __init__ (self.) so its state survives between update() calls.
  # rc = tau [s]: reaches 63% of a step after tau, ~95% after 3*tau.
  #   rc=0.05 -> kills most frame-to-frame noise, adds ~150 ms to full response. Tune in car.
  # Activate together with the import at the top, AFTER the comparable unit scale (*3?) is confirmed.
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.driver_torque_filter = FirstOrderFilter(0., 0.05, DT_CTRL)
    self.eps_state_lka = 0
    self.speed_kph = 0.0
    self.artiv_diag_response_updated = False
    self.artiv_diag_response = {
      "ISO_TP_LENGTH": 0,
      "UDS_SERVICE": 0,
      "UDS_SUBFUNCTION": 0,
      "UDS_NRC": 0,
    }

  # #HANDS-FREE - START: state for the EPS silent-dropout safety net
  # def __init__(self, CP, CP_SP):
  #   super().__init__(CP, CP_SP)
  #   self.eps_state_last = 0
  #   self.eps_fault_frames = 0
  # #HANDS-FREE - END

  # def update(self, can_parsers) -> structs.CarState:
  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.main]
    cp_adas = can_parsers[Bus.adas]
    cp_cam = can_parsers[Bus.cam]
    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    # car speed
    self.parse_wheel_speeds(ret,
      cp.vl['Dyn4_FRE']['P263_VehV_VPsvValWhlFrtL'],
      cp.vl['Dyn4_FRE']['P264_VehV_VPsvValWhlFrtR'],
      cp.vl['Dyn4_FRE']['P265_VehV_VPsvValWhlBckL'],
      cp.vl['Dyn4_FRE']['P266_VehV_VPsvValWhlBckR'],
    )
    ret.yawRate = cp_adas.vl['HS2_DYN_UCF_MDD_32D']['VITESSE_LACET_BRUTE'] * CV.DEG_TO_RAD
    if self.CP.carFingerprint in (CAR.PSA_PEUGEOT_3008, CAR.PSA_CITROEN_C4_SPACETOURER):
      # ret.standstill = ret.vEgoRaw <= 0
      ret.standstill = ret.vEgoRaw <= 0.1 * CV.KPH_TO_MS
      self.speed_kph = ret.vEgoRaw * CV.MS_TO_KPH
    else:
      # versione elkoled
      ret.standstill = bool(cp_adas.vl['HS2_DYN_UCF_MDD_32D']['VEHICLE_STANDSTILL'])
    # [CLAUDE standstill-threshold] - END

    # gas
    if self.CP.carFingerprint in (CAR.PSA_PEUGEOT_3008, CAR.PSA_CITROEN_C4_SPACETOURER):
      ret.gasPressed = cp.vl['Dyn5_CMM']['P334_ACCPed_Position'] > 0
    else:
      ret.gasPressed = cp_cam.vl['DRIVER']['GAS_PEDAL'] > 0

    # brake pressed
    ret.brakePressed = bool(cp_cam.vl['Dat_BSI']['P013_MainBrake'])

    # brake pressure

    # parking brake
    ret.parkingBrake = cp.vl['Dyn_EasyMove']['P337_Com_stPrkBrk'] == 1 # 0: disengaged, 1: engaged, 3: brake actuator moving

    # steering wheel
    STEERING_ALT_BUS = {
      CAR.PSA_PEUGEOT_208: cp.vl,
      CAR.PSA_PEUGEOT_508: cp_cam.vl,
      CAR.PSA_PEUGEOT_3008: cp.vl,
      CAR.PSA_CITROEN_C4_SPACETOURER: cp.vl,
    }
    bus = STEERING_ALT_BUS[self.CP.carFingerprint]
    ret.steeringAngleDeg = bus['STEERING_ALT']['ANGLE'] # EPS
    if self.CP.carFingerprint in ( CAR.PSA_PEUGEOT_3008, CAR.PSA_CITROEN_C4_SPACETOURER ):
      # PSA EPS encodes the steering rotation direction bit inverted from the driver's perspective:
      #   RATE_SIGN = 0 → clockwise (right turn)
      #   RATE_SIGN = 1 → anticlockwise (left turn)
      # Invert the sign to match OpenPilot's convention: right = positive, left = negative.
      ret.steeringRateDeg = bus['STEERING_ALT']['RATE'] * (1 - 2 * bus['STEERING_ALT']['RATE_SIGN'])
    else:
      # Convert EPS direction bit [0,1] to signed multiplier [-1,+1]
      # Standard convention: 0 → left (negative), 1 → right (positive)
      ret.steeringRateDeg  = bus['STEERING_ALT']['RATE'] * (2 * bus['STEERING_ALT']['RATE_SIGN'] - 1)

    if self.CP.carFingerprint in (CAR.PSA_PEUGEOT_3008, CAR.PSA_CITROEN_C4_SPACETOURER ):
      ret.genericToggle = (int(cp.vl["IS_DAT_DIRA"]["ETAT_DA_DYN"]) == 1) # 0 = Normal, 1 = Dynamic/Sport, 2 = Adjustable

      # ret.steeringTorque  = cp.vl['STEERING']['DRIVER_TORQUE'] * 3   # raw (noisy) - sostituita dalla versione filtrata sotto
      # ret.steeringTorqueEps = 0.0
      # Low-pass sul DRIVER_TORQUE rumoroso (FirstOrderFilter, rc=0.05s ~150ms):
      ret.steeringTorque = self.driver_torque_filter.update(cp.vl['STEERING']['DRIVER_TORQUE']) * 3
      ret.steeringTorqueEps = cp.vl['IS_DAT_DIRA']['EPS_TORQUE'] * 10 * 3

    else:
      ret.steeringTorque = cp.vl['STEERING']['DRIVER_TORQUE']
      ret.steeringTorqueEps = cp.vl['IS_DAT_DIRA']['EPS_TORQUE']

    if self.CP.carFingerprint in ( CAR.PSA_PEUGEOT_3008, CAR.PSA_CITROEN_C4_SPACETOURER ):
      # Peugeot 3008: EPS_TORQUE represents only driver-applied torque (no motor assist).
      # The signal is already smoothed by the EPS ECU, so update_steering_pressed is unnecessary.
      # ret.steeringPressed = abs(ret.steeringTorque) > LKAS_LIMITS.STEER_THRESHOLD
      # ret.steeringPressed = abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE
      ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE, 5)
    else:
      ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE, 5)

    self.eps_active = cp.vl['IS_DAT_DIRA']['EPS_STATE_LKA'] == 3 # 0: Unauthorized, 1: Authorized, 2: Available, 3: Active, 4: Defect
    # [CLAUDE eps-closed-loop] - START
    # Valore grezzo, non solo "e' Active": la scaletta di riattivazione sale di
    # gradino solo quando l'EPS ha confermato quello precedente (vedi EPS_STATUS_ACK
    # in carcontroller.py).
    self.eps_state_lka = int(cp.vl['IS_DAT_DIRA']['EPS_STATE_LKA'])
    # [CLAUDE eps-closed-loop] - END
    self.is_dat_dira = copy.copy(cp.vl['IS_DAT_DIRA'])
    self.steering = copy.copy(cp.vl['STEERING'])
    self.HS2_DYN_MDD_ETAT_2F6 =copy.copy(cp_adas.vl['HS2_DYN_MDD_ETAT_2F6'])

    # Risposta diagnostica ARTIV su 0x696. I quattro segnali bastano sia per
    # una risposta positiva 50 xx sia per quella negativa 7F 10 <NRC>.
    # Profilo ECU: https://github.com/Barracuda09/PyPSADiag/blob/main/json/ARTIV/ARTIV_UDS.json
    artiv_services = cp_adas.vl_all["Rep_Diag_ARTIV"]["UDS_SERVICE"]
    self.artiv_diag_response_updated = len(artiv_services) > 0
    if self.artiv_diag_response_updated:
      self.artiv_diag_response = {
        signal: int(cp_adas.vl_all["Rep_Diag_ARTIV"][signal][-1])
        for signal in self.artiv_diag_response
      }

    # cruise
    ret.cruiseState.speed = cp_adas.vl['HS2_DAT_MDD_CMD_452']['SPEED_SETPOINT'] * CV.KPH_TO_MS # set to 255 when ACC is off, -2 kph offset from dash speed
    ret.cruiseState.enabled = cp_adas.vl['HS2_DAT_MDD_CMD_452']['RVV_ACC_ACTIVATION_REQ'] == 1
    ret.cruiseState.available = True # not available for CC-only
    ret.cruiseState.nonAdaptive = False # not available for CC-only

    ret.cruiseState.standstill = False # not available for CC-only
    ret.accFaulted = cp_adas.vl['HS2_DYN_UCF_MDD_32D']['ACC_ETAT_DECEL_OR_ESP_STATUS'] == 3
    self.hs2_dat_mdd_cmd_452 = copy.copy(cp_adas.vl['HS2_DAT_MDD_CMD_452'])

    # gear
    if bool(cp_cam.vl['Dat_BSI']['P103_Com_bRevGear']):
      ret.gearShifter = GearShifter.reverse
    else:
      ret.gearShifter = GearShifter.drive

    # blinkers
    blinker = cp_cam.vl['HS2_DAT7_BSI_612']['CDE_CLG_ET_HDC']
    if self.CP.carFingerprint in( CAR.PSA_PEUGEOT_3008, CAR.PSA_CITROEN_C4_SPACETOURER ):
      ret.leftBlinker = blinker == 2
      ret.rightBlinker = blinker == 1
    else:
      ret.leftBlinker = blinker == 1
      ret.rightBlinker = blinker == 2

    # Blind sensor ( there is not left and right )
    if self.CP.carFingerprint in( CAR.PSA_PEUGEOT_3008, CAR.PSA_CITROEN_C4_SPACETOURER ):
      ret.leftBlindspot = cp_adas.vl["HS2_DYN_MDD_ETAT_2F6"]["BLIND_SENSOR"] != 0
      ret.rightBlindspot = cp_adas.vl["HS2_DYN_MDD_ETAT_2F6"]["BLIND_SENSOR"] != 0

    # Auto Braking in progress
    if self.CP.carFingerprint in( CAR.PSA_PEUGEOT_3008, CAR.PSA_CITROEN_C4_SPACETOURER ):
      ret.stockAeb = cp_adas.vl["HS2_DYN1_MDD_ETAT_2B6"]["AUTO_BRAKING_STATUS"] == 1

    # lock info
    ret.doorOpen = any((cp_cam.vl['Dat_BSI']['DRIVER_DOOR'], cp_cam.vl['Dat_BSI']['PASSENGER_DOOR']))
    ret.seatbeltUnlatched = cp_cam.vl['RESTRAINTS']['DRIVER_SEATBELT'] != 2

    return ret, ret_sp

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    return {
      Bus.main: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
      # Evento diagnostico: math.nan lo registra nel parser senza renderlo
      # obbligatorio per canValid quando non stiamo eseguendo il test ARTIV.
      Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.pt], [("Rep_Diag_ARTIV", math.nan)], 1),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
