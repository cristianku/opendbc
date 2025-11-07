# import random
import math

# ELKOLED VERSION
def psa_checksum(address: int, sig, d: bytearray) -> int:
  chk_ini = {0x452: 0x4, 0x38D: 0x7, 0x42D: 0xC}.get(address, 0xB)
  byte = sig.start_bit // 8
  d[byte] &= 0x0F if sig.start_bit % 8 >= 4 else 0xF0
  checksum = sum((b >> 4) + (b & 0xF) for b in d)
  return (chk_ini - checksum) & 0xF


# ## ELKOLED VERSION + 2F6
# def psa_checksum(address: int, sig, d: bytearray) -> int:
#   chk_ini = {0x452: 0x4, 0x38D: 0x7, 0x42D: 0xC, 0x2F6: 0x8}.get(address, 0xB)
#   byte = sig.start_bit // 8
#   d[byte] &= 0x0F if sig.start_bit % 8 >= 4 else 0xF0
#   checksum = sum((b >> 4) + (b & 0xF) for b in d)
#   return (chk_ini - checksum) & 0xF

#### USE THIS TO CHECK THE CHECKSUM AGAINS REAL DATA https://github.com/cristianku/opendbc-checksum-tools
#CRISTIANKU VERSION
# def psa_checksum(address: int, sig, d: bytearray) -> int:
#   # Skip disabled checksums (prefix "0_")
#   if sig.name.startswith("0_"):
#     return 0

#   # HS2_DAT_MDD_CMD_452 1106 / 0x452
#   if address == 0x452 and  sig.name == "CHECKSUM_CONS_RVV_LVV2":
#     # Extract SPEED_SETPOINT from byte 1
#     speed_setpoint = d[1]
#     # Calculate parity of each nibble
#     msb_nibble = (speed_setpoint >> 4) & 0xF
#     lsb_nibble = speed_setpoint & 0xF
#     # Count bits (parity): 1 if odd number of 1s, 0 if even
#     msb_parity = bin(msb_nibble).count('1') & 1
#     lsb_parity = bin(lsb_nibble).count('1') & 1
#     # Return 2-bit checksum: [msb_parity, lsb_parity]
#     return (msb_parity << 1) | lsb_parity

#   # --- SPECIAL CASE: 0x305 (STEERING_ALT 773) ---
#   if address == 0x305:
#     # "checksum" is just the upper nibble of byte 4
#     return (d[4] >> 4) & 0xF

#   chk_ini = {
#              0x1CD: 0x5,  # 461 decimal - ESP
#              0x2B6: 0xC,  # 694 decimal - HS2_DYN1_MDD_ETAT_2B6 - override 0xC su ECU MDD 2018+)
#              0x2F6: 0x8,  # 758 decimal - message ACC2
#              0x38D: 0x7,  #  909 - HS2_DYN_ABR_38D
#              0x42D: 0xC,  # 1069 - NEW_MSG_42D
#              0x452: 0x4   # 1106 - HS2_DAT_MDD_CMD_452
#              }.get(address, 0xB)

#   byte = sig.start_bit // 8
#   d[byte] &= 0x0F if sig.start_bit % 8 >= 4 else 0xF0

#   checksum = sum((b >> 4) + (b & 0xF) for b in d)
#   return (chk_ini - checksum) & 0xF



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


# def create_lka_steering(packer, lat_active: bool, apply_torque: float, torque_factor: int, status: int):
def create_lka_steering(packer, lane_keep_assist, lat_active: bool, apply_torque: float, torque_factor: int, status: int):
  # values = {
  #   'TORQUE': apply_torque,
  #   # 'LANE_DEPARTURE':0 if not lat_active else 1 if torque>0 else 2,
  #   # 'DRIVE': 1,
  #   'STATUS': status,
  #   # 'LXA_ACTIVATION': 1,
  #   'TORQUE_FACTOR': torque_factor, # * 100,
  #   'SET_ANGLE': 0,
  # }
  lane_keep_assist['TORQUE'] = apply_torque
  lane_keep_assist['STATUS'] = status
  lane_keep_assist['TORQUE_FACTOR'] = torque_factor
  lane_keep_assist['SET_ANGLE'] = 0

  # return packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)
  return packer.make_can_msg('LANE_KEEP_ASSIST', 0, lane_keep_assist)


# def create_driver_torque(packer, steering):
#   # abs(driver_torque) > 10 to keep EPS engaged
#   torque = steering['DRIVER_TORQUE']

#   if abs(torque) < 10:
#     steering['DRIVER_TORQUE'] = 10 if torque > 0 else -10

#   return packer.make_can_msg('STEERING', 0, steering)

# def driver_torque_from_eps(eps: float) -> int:
#     # eps quantizzato a 0.25
#     return round(9.6 * eps)
def convert_driver_torque_to_eps(driver_torque: float) -> float:
    """
    Convert driver torque to EPS torque.
    Formula: divide by 10 and quantize to 0.25 Nm (floor)
    """
    # # Divide by 10 and quantize to 0.25 Nm (floor)
    # eps_torque = driver_torque / 10.0
    # quantized = math.floor(eps_torque / 0.25) * 0.25
    eps_torque = math.floor(driver_torque / 2.5) * 2.5 / 10

    return round(eps_torque, 2)

def create_driver_torque(packer, steering, driver_torque):
  steering['DRIVER_TORQUE'] = driver_torque
  return packer.make_can_msg('STEERING', 0, steering)

def relay_driver_torque(packer, steering):
  """Relay DRIVER_TORQUE unchanged (passthrough to prevent Panda forwarding)."""
  return packer.make_can_msg('STEERING', 0, steering)


def create_steering_hold(packer, is_dat_dira, driver_torque):
  """
  Crea messaggio IS_DAT_DIRA con EPS_TORQUE e STEERWHL_HOLD_BY_DRV.

  Args:
      packer: CANPacker instance
      is_dat_dira: dict con i valori base di IS_DAT_DIRA
      eps_torque: EPS torque già filtrato e convertito (da EPSTorqueFilter)

  Returns:
      CAN message per IS_DAT_DIRA
  """
  if driver_torque > 6:
    is_dat_dira['STEERWHL_HOLD_BY_DRV'] = 1
  else:
    is_dat_dira['STEERWHL_HOLD_BY_DRV'] = 0
    # eps_converter.convert_driver_to_hold(driver_torque)

  # Convert driver torque to EPS torque scale (with quantization)
  is_dat_dira['EPS_TORQUE'] = convert_driver_torque_to_eps(driver_torque)


  return packer.make_can_msg('IS_DAT_DIRA', 2, is_dat_dira)

def relay_is_dat_dira(packer, is_dat_dira,driver_torque):
  return packer.make_can_msg('IS_DAT_DIRA', 2, is_dat_dira)

def create_request_takeover(packer, HS2_DYN_MDD_ETAT_2F6, type):
  # HS2_DYN_MDD_ETAT_2F6
  #  1 = Non Critical Request
  #  2 = Critical request
  HS2_DYN_MDD_ETAT_2F6['REQUEST_TAKEOVER'] = type

  return packer.make_can_msg('HS2_DYN_MDD_ETAT_2F6', 1, HS2_DYN_MDD_ETAT_2F6)

def create_wheel_speed_spoof(packer, dyn4_fre, min_speed=55.0):
  """
  Spoof wheel speeds to keep EPS active by ensuring speed never drops below min_speed.

  Args:
      packer: CANPacker instance
      dyn4_fre: dict with original Dyn4_FRE values
      min_speed: minimum speed in km/h (default 55 km/h to keep EPS active)

  Returns:
      CAN message for Dyn4_FRE on bus 0
  """
  dyn4_fre_spoofed = dict(dyn4_fre)
  dyn4_fre_spoofed['P263_VehV_VPsvValWhlFrtL'] = max(min_speed, dyn4_fre['P263_VehV_VPsvValWhlFrtL'])
  dyn4_fre_spoofed['P264_VehV_VPsvValWhlFrtR'] = max(min_speed, dyn4_fre['P264_VehV_VPsvValWhlFrtR'])
  dyn4_fre_spoofed['P265_VehV_VPsvValWhlBckL'] = max(min_speed, dyn4_fre['P265_VehV_VPsvValWhlBckL'])
  dyn4_fre_spoofed['P266_VehV_VPsvValWhlBckR'] = max(min_speed, dyn4_fre['P266_VehV_VPsvValWhlBckR'])

  return packer.make_can_msg('Dyn4_FRE', 0, dyn4_fre_spoofed)

  # Bus.main: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
  # Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 1),
  # Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),

