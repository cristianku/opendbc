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


# [inactive lka] - START
# <TEST_ANGLE_START>
# def create_lka_steering(packer, lat_active: bool, apply_torque: float, torque_factor: int, status: int, *, unknown2: int):
#   values = {
#     'unknown2': unknown2,
#     'TORQUE': apply_torque,
#     'STATUS': status,
#     'TORQUE_FACTOR': torque_factor,
#   }
#   return packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)
def create_lka_steering(packer, lat_active: bool, apply_torque: float, torque_factor: int, status: int, *, unknown2: int,
                        drive: int, lxa_activation: int, set_angle: float, counter: int | None):
  values = {
    'unknown2': unknown2,
    'TORQUE': apply_torque,
    # 'LANE_DEPARTURE':0 if not lat_active else 1 if torque>0 else 2,
    'DRIVE': drive,
    'STATUS': status,
    'LXA_ACTIVATION': lxa_activation,
    'TORQUE_FACTOR': torque_factor, # * 100,
    'SET_ANGLE': set_angle,
  }

  # None explicitly retains the legacy torque payload (zero counter/checksum).
  # The prefixed DBC names are not processed automatically by CANPacker.
  if counter is not None:
    values['0_COUNTER'] = counter
    data = bytearray(packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)[1])
    checksum_sig = packer.dbc.name_to_msg['LANE_KEEP_ASSIST'].sigs['0_CHECKSUM']
    values['0_CHECKSUM'] = psa_checksum(0x3F2, checksum_sig, data)
  return packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)
# <TEST_ANGLE_START_END>
# [inactive lka] - END


# def create_driver_torque(packer, steering):
#   # abs(driver_torque) > 10 to keep EPS engaged
#   torque = steering['DRIVER_TORQUE']

#   if abs(torque) < 10:
#     steering['DRIVER_TORQUE'] = 10 if torque > 0 else -10

#   return packer.make_can_msg('STEERING', 0, steering)
# def create_resume_acc(packer, counter, status, hs2_dat_mdd_cmd_452):
#   hs2_dat_mdd_cmd_452['COUNTER'] = counter
#   hs2_dat_mdd_cmd_452['COCKPIT_GO_ACC_REQUEST'] = status
#   return packer.make_can_msg('HS2_DAT_MDD_CMD_452', 1, hs2_dat_mdd_cmd_452)


def create_drive_away_request(packer, hs2_dyn_mdd_etat_2f6):
  hs2_dyn_mdd_etat_2f6['DRIVE_AWAY_REQUEST'] = 0
  return packer.make_can_msg('HS2_DYN_MDD_ETAT_2F6', 1, hs2_dyn_mdd_etat_2f6)


def create_HS2_DYN1_MDD_ETAT_2B6(packer, bus: int, *, mdd_desired_deceleration: float,
                               potential_wheel_torque_request: int, min_time_for_desired_gear: float,
                               gmp_potential_wheel_torque: float, acc_status: int, gmp_wheel_torque: float,
                               wheel_torque_request: int, auto_braking_status: int, mdd_decel_type: int,
                               mdd_decel_control_req: int, gear_type: int, prefill_request: int, counter: int):
  values = {
    'MDD_DESIRED_DECELERATION': mdd_desired_deceleration,
    'POTENTIAL_WHEEL_TORQUE_REQUEST': potential_wheel_torque_request,
    'MIN_TIME_FOR_DESIRED_GEAR': min_time_for_desired_gear,
    'GMP_POTENTIAL_WHEEL_TORQUE': gmp_potential_wheel_torque,
    'ACC_STATUS': acc_status,
    'GMP_WHEEL_TORQUE': gmp_wheel_torque,
    'WHEEL_TORQUE_REQUEST': wheel_torque_request,
    'AUTO_BRAKING_STATUS': auto_braking_status,
    'MDD_DECEL_TYPE': mdd_decel_type,
    'MDD_DECEL_CONTROL_REQ': mdd_decel_control_req,
    'GEAR_TYPE': gear_type,
    'PREFILL_REQUEST': prefill_request,
    'COUNTER': counter,
  }
  return packer.make_can_msg('HS2_DYN1_MDD_ETAT_2B6', bus, values)


def create_HS2_DYN_MDD_ETAT_2F6(packer, bus: int, *, target_detected: int, request_takeover: int, blind_sensor: int,
                              req_visual_coll_alert_arc: int, req_audio_coll_alert_arc: int, req_haptic_coll_alert_arc: int,
                              inter_vehicle_distance: float, arc_status: int, auto_braking_in_progress: int, aeb_enabled: int,
                              drive_away_request: int, display_intervehicle_time: float, mdd_decel_control_req: int,
                              auto_braking_status: int, counter: int, target_position: int):
  values = {
    'TARGET_DETECTED': target_detected,
    'REQUEST_TAKEOVER': request_takeover,
    'BLIND_SENSOR': blind_sensor,
    'REQ_VISUAL_COLL_ALERT_ARC': req_visual_coll_alert_arc,
    'REQ_AUDIO_COLL_ALERT_ARC': req_audio_coll_alert_arc,
    'REQ_HAPTIC_COLL_ALERT_ARC': req_haptic_coll_alert_arc,
    'INTER_VEHICLE_DISTANCE': inter_vehicle_distance,
    'ARC_STATUS': arc_status,
    'AUTO_BRAKING_IN_PROGRESS': auto_braking_in_progress,
    'AEB_ENABLED': aeb_enabled,
    'DRIVE_AWAY_REQUEST': drive_away_request,
    'DISPLAY_INTERVEHICLE_TIME': display_intervehicle_time,
    'MDD_DECEL_CONTROL_REQ': mdd_decel_control_req,
    'AUTO_BRAKING_STATUS': auto_braking_status,
    'COUNTER': counter,
    'TARGET_POSITION': target_position,
  }
  return packer.make_can_msg('HS2_DYN_MDD_ETAT_2F6', bus, values)


def create_HS2_DAT_ARTIV_V2_4F6(packer, bus: int, *, time_gap: float, distance_gap: float, relative_speed: float,
                              artiv_sensor_state: int, target_detected: int, artiv_target_change_info: int, traffic_direction: int):
  values = {
    'TIME_GAP': time_gap,
    'DISTANCE_GAP': distance_gap,
    'RELATIVE_SPEED': relative_speed,
    'ARTIV_SENSOR_STATE': artiv_sensor_state,
    'TARGET_DETECTED': target_detected,
    'ARTIV_TARGET_CHANGE_INFO': artiv_target_change_info,
    'TRAFFIC_DIRECTION': traffic_direction,
  }
  return packer.make_can_msg('HS2_DAT_ARTIV_V2_4F6', bus, values)


def create_HS2_SUPV_ARTIV_796(packer, bus: int, *, fault_code: int, status_no_config: int,
                            status_partial_wakeup_gmp: int, uce_electr_state: int):
  values = {
    'FAULT_CODE': fault_code,
    'STATUS_NO_CONFIG': status_no_config,
    'STATUS_PARTIAL_WAKEUP_GMP': status_partial_wakeup_gmp,
    'UCE_ELECTR_STATE': uce_electr_state,
  }
  return packer.make_can_msg('HS2_SUPV_ARTIV_796', bus, values)

# def create_driver_torque(packer, steering, counter):
#   #0x2F5 message
#   t = int(steering.get('DRIVER_TORQUE', 0))
#   if abs(t) < 10:
#     t = random.randint(10, 12)
#   t = max(0, min(20, t))
#   steering['DRIVER_TORQUE'] = t
#   steering['COUNTER'] = counter
#   return packer.make_can_msg('STEERING', 0, steering)

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
# Richiede la programming session ARTIV su 0x6B6, bus ADAS.
# Riferimento per gli indirizzi ECU (non prova dell'accettazione della sessione):
# https://github.com/Barracuda09/PyPSADiag/blob/main/json/ARTIV/ARTIV_UDS.json
#   0x02 = ISO-TP single frame, 2 byte di payload
#   0x10 0x02 = DiagnosticSessionControl, programmingSession.
# DLC 3: il radar ha risposto a TesterPresent corto, non a quello con padding.
# Verificare su 0x696 la risposta 50 02 oppure 7F 10 xx e osservare i frame radar.
# Accettazione, sospensione dei messaggi e ritorno alla sessione normale sono da
# verificare sul veicolo; questa prova non invia keepalive, erase o write.
def create_disable_radar():
  # https://github.com/ludwig-v/arduino-psa-diag/blob/master/ECU_LIST.md
  addr = 0x6B6
  return CanData(addr, b'\x02\x10\x02', PSA_ADAS_BUS)
# [artiv-diag-probe] - END
