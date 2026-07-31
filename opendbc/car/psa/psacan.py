import random

from opendbc.car.can_definitions import CanData
from opendbc.car.psa.values import PSA_ADAS_BUS


def psa_checksum(address: int, sig, d: bytearray) -> int:
  chk_ini = {0x452: 0x4, 0x38D: 0x7, 0x2f6: 0x8, 0x2b6: 0xC, 0x42D: 0xC}.get(address, 0xB)
  byte = sig.start_bit // 8
  d[byte] &= 0x0F if sig.start_bit % 8 >= 4 else 0xF0
  checksum = sum((b >> 4) + (b & 0xF) for b in d)
  return (chk_ini - checksum) & 0xF


# def create_lka_steering(packer, apply_torque: int, torque_factor: int, status: int):
#   values = {
#     'TORQUE': apply_torque ,
#     # 'LANE_DEPARTURE':0 if not lat_active else 1 if torque>0 else 2,
#     # 'DRIVE': 1,
#     'STATUS': status,
#     # 'LXA_ACTIVATION': 1,
#     'TORQUE_FACTOR': torque_factor,
#     'SET_ANGLE': 0,
#   }

#   return packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)


def create_lka_steering(packer, lat_active: bool, apply_torque: float, torque_factor: int, status: int, set_angle: int):
  values = {
    'unknown2': 24,
    'TORQUE': apply_torque,
    # 'LANE_DEPARTURE':0 if not lat_active else 1 if torque>0 else 2,
    # 'DRIVE': 1,
    'STATUS': status,
    # 'LXA_ACTIVATION': 1,
    'TORQUE_FACTOR': torque_factor, # * 100,
    'SET_ANGLE': set_angle,
  }

  return packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)


# def create_driver_torque(packer, steering):
#   # abs(driver_torque) > 10 to keep EPS engaged
#   torque = steering['DRIVER_TORQUE']

#   if abs(torque) < 10:
#     steering['DRIVER_TORQUE'] = 10 if torque > 0 else -10

#   return packer.make_can_msg('STEERING', 0, steering)
def create_resume_acc(packer, counter, status, hs2_dat_mdd_cmd_452):
  hs2_dat_mdd_cmd_452['COUNTER'] = counter
  hs2_dat_mdd_cmd_452['COCKPIT_GO_ACC_REQUEST'] = status
  return packer.make_can_msg('HS2_DAT_MDD_CMD_452', 1, hs2_dat_mdd_cmd_452)


def create_drive_away_request(packer, hs2_dyn_mdd_etat_2f6):
  hs2_dyn_mdd_etat_2f6['DRIVE_AWAY_REQUEST'] = 0
  return packer.make_can_msg('HS2_DYN_MDD_ETAT_2F6', 1, hs2_dyn_mdd_etat_2f6)


# Radar, 50 Hz
def create_HS2_DYN1_MDD_ETAT_2B6(packer, frame: int, accel: float, enabled: bool, gasPressed: bool,
                                 braking: bool, brakePressed: bool, standstill: bool, torque: int):
  # TODO: if gas pressed, ACC_STATUS is set to suspended and decel can be set negative (about -300 Nm / -0.6m/s²) with brake mode inactive
  # TODO: tune torque multiplier
  # TODO: check difference between GMP_POTENTIAL_WHEEL_TORQUE and GMP_WHEEL_TORQUE
  # TODO: transition from waiting to active enables torque control. For now, deactivate autohold or enable on brake pressed

  values = {
    'MDD_DESIRED_DECELERATION': accel if braking and enabled else 2.05, # m/s²
    'POTENTIAL_WHEEL_TORQUE_REQUEST': (2 if braking else 1) if enabled else 0,
    'MIN_TIME_FOR_DESIRED_GEAR': 0.0 if braking or not enabled else 6.2,
    'GMP_POTENTIAL_WHEEL_TORQUE': torque if not braking and enabled else -4000,
    'ACC_STATUS': (5 if gasPressed else 2 if brakePressed and not standstill else 4) if enabled else (2 if brakePressed else 3),
    'GMP_WHEEL_TORQUE': torque if not braking and enabled else -4000,
    'WHEEL_TORQUE_REQUEST': 1 if enabled and not braking else 0, # TODO: test 1: high torque range 2: low torque range
    'AUTO_BRAKING_STATUS': 3, # AEB # TODO: testing ALWAYS ENABLED to resolve DTC errors if enabled else 3, # maybe disabled on too high steering angle
    'MDD_DECEL_TYPE': braking if enabled else 0,
    'MDD_DECEL_CONTROL_REQ': braking if enabled else 0,
  }

  return packer.make_can_msg('HS2_DYN1_MDD_ETAT_2B6', 1, values)


# Radar, 50 Hz
def create_HS2_DYN_MDD_ETAT_2F6(packer, braking: bool, lead_visible: bool, lead_distance_bars: int):
  values = {
    'TARGET_DETECTED': lead_visible,
    # 'REQUEST_TAKEOVER': 0, # TODO potential signal for HUD message from OP
    # 'BLIND_SENSOR': 0,
    # 'REQ_VISUAL_COLL_ALERT_ARC': 0,
    # 'REQ_AUDIO_COLL_ALERT_ARC': 0,
    # 'REQ_HAPTIC_COLL_ALERT_ARC': 0,
    # 'INTER_VEHICLE_DISTANCE': 255.5,#255.5, # TODO: <distance> if enabled else 255.5,
    # 'ARC_STATUS': 6,  # 12 after 50 frames (1 sec) after AUTO_BRAKING_STATUS else 6
    # 'AUTO_BRAKING_IN_PROGRESS': 0,
    # 'AEB_ENABLED': 0,
    # 'DRIVE_AWAY_REQUEST': 0, # TODO: potential RESUME request?
    'DISPLAY_INTERVEHICLE_TIME': 5.0, # TODO: <time to vehicle> if enabled else 6.2,
    'MDD_DECEL_CONTROL_REQ': braking,
    # 'AUTO_BRAKING_STATUS': 3, # AEB # TODO: testing ALWAYS ENABLED to resolve DTC errors if enabled else 3, # maybe disabled on too high steering angle
    'TARGET_POSITION': lead_distance_bars, # distance to lead car, far - 4, 3, 2, 1 - near
  }

  return packer.make_can_msg('HS2_DYN_MDD_ETAT_2F6', 1, values)

def create_driver_torque(packer, steering, counter):
  t = int(steering.get('DRIVER_TORQUE', 0))
  if abs(t) < 10:
    t = random.randint(10, 12)
  t = max(0, min(20, t))
  steering['DRIVER_TORQUE'] = t
  steering['COUNTER'] = counter
  return packer.make_can_msg('STEERING', 0, steering)

def create_steering_hold(packer, lat_active: bool, is_dat_dira):
  # set STEERWHL_HOLD_BY_DRV to keep EPS engaged when lat active
  if lat_active:
    is_dat_dira['STEERWHL_HOLD_BY_DRV'] = 1
  return packer.make_can_msg('IS_DAT_DIRA', 2, is_dat_dira)

def create_request_takeover(packer, HS2_DYN_MDD_ETAT_2F6, takeover_type):
  HS2_DYN_MDD_ETAT_2F6['REQUEST_TAKEOVER'] = takeover_type
  return packer.make_can_msg('HS2_DYN_MDD_ETAT_2F6', 1, HS2_DYN_MDD_ETAT_2F6)

# def set_speed(packer, hs2_dat_mdd_cmd_452, speed_kph: float):
#   values = dict(hs2_dat_mdd_cmd_452)
#   speed_setpoint = max(0, min(255, round(speed_kph)))
#   values['SPEED_SETPOINT'] = speed_setpoint

#   # Encode the bit parity of each nibble: bit 1 for odd parity in the most
#   # significant nibble, bit 0 for odd parity in the least significant nibble.
#   ms_nibble_parity = ((speed_setpoint >> 4) & 0xF).bit_count() & 1
#   ls_nibble_parity = (speed_setpoint & 0xF).bit_count() & 1
#   values["CHECKSUM_CONS_RVV_LVV2"] = (ms_nibble_parity << 1) | ls_nibble_parity
#   return packer.make_can_msg('HS2_DAT_MDD_CMD_452', 1, values)

# [artiv-diag-probe] - START
# Disabilita ARTIV (radar) mettendolo in programming session su 0x6B6, bus ADAS.
# Riferimento profilo ECU PyPSADiag:
# https://github.com/Barracuda09/PyPSADiag/blob/main/json/ARTIV/ARTIV_UDS.json
#   0x02 = ISO-TP single frame, 2 byte di payload
#   0x10 0x02 = DiagnosticSessionControl, programmingSession -> l'ECU smette di
#               trasmettere i suoi messaggi normali (radar "zitto") finche' resta li'.
# La sessione decade dopo il timeout S3 (~5 s): il CarController la tiene viva con
# TesterPresent periodico. Entrare in programming NON richiede security access (27);
# quello servirebbe solo per erase/write. Risposta lasciata attiva (50 02 vs 7F 10 xx)
# per vedere nei log se ARTIV ha accettato; per sopprimerla userei 0x10 0x82.
#
# La safety autorizza 0x6B6 con DLC 8. Il primo byte ISO-TP dichiara comunque
# due byte UDS; i cinque byte rimanenti sono padding a zero.
def create_disable_radar():
  # https://github.com/ludwig-v/arduino-psa-diag/blob/master/ECU_LIST.md
  addr = 0x6B6
  dat = [0x02, 0x10, 0x02]
  dat.extend([0x0] * (8 - len(dat)))

  return CanData(addr, bytes(dat), PSA_ADAS_BUS)
