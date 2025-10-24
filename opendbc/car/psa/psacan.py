from opendbc.car.lateral import apply_driver_steer_torque_limits

def psa_checksum(address: int, sig, d: bytearray) -> int:
  chk_ini = {0x452: 0x4, 0x38D: 0x7, 0x42D: 0xC}.get(address, 0xB)
  byte = sig.start_bit // 8
  d[byte] &= 0x0F if sig.start_bit % 8 >= 4 else 0xF0
  checksum = sum((b >> 4) + (b & 0xF) for b in d)
  return (chk_ini - checksum) & 0xF

def create_lka_steering(packer, apply_torque: int, torque_factor: int, status: int):
  values = {
    'TORQUE': apply_torque ,
    # 'LANE_DEPARTURE':0 if not lat_active else 1 if torque>0 else 2,
    # 'DRIVE': 1,
    'STATUS': status,
    # 'LXA_ACTIVATION': 1,
    'TORQUE_FACTOR': torque_factor,
    'SET_ANGLE': 0,
  }

  return packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)

def calculate_apply_torque( actuators_torque, driver_torque, apply_torque_last, CarControllerParams):
  temp_new_torque = int(round(actuators_torque * CarControllerParams.STEER_MAX))

  # apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
  #                                                 CS.out.steeringTorque, CarControllerParams)

  return apply_driver_steer_torque_limits(temp_new_torque, apply_torque_last,
                                                  driver_torque, CarControllerParams )

  return apply_driver_steer_torque_limits(temp_new_torque, apply_torque_last,
                                                  0, CarControllerParams )

def calculate_apply_factor(previous_factor):
  apply_torque_factor = previous_factor + 5

  if apply_torque_factor > 100:
      apply_torque_factor = 100

  return apply_torque_factor

def calculate_LKA_status(lat_active:bool, eps_active:bool, actual_status):
    new_status = actual_status
    # Openpilot is not activated
    if not lat_active:
      new_status =  2
    # elif not CS.eps_active and not CS.out.steeringPressed:
    # Cycling to activate the EPS
    elif not eps_active:
      if actual_status == 4:
        new_status = 2
      else:
        new_status += 1
    # The EPS is already active, no need to cycle for activation
    else:
      new_status =  4

    return new_status


def create_driver_torque(packer, steering):
  # abs(driver_torque) > 10 to keep EPS engaged
  torque = steering['DRIVER_TORQUE']

  if abs(torque) < 10:
    steering['DRIVER_TORQUE'] = 10 if torque > 0 else -10

  return packer.make_can_msg('STEERING', 0, steering)


def create_steering_hold(packer, lat_active: bool, is_dat_dira):
  # set STEERWHL_HOLD_BY_DRV to keep EPS engaged when lat active
  if lat_active:
    is_dat_dira['STEERWHL_HOLD_BY_DRV'] = 1
  return packer.make_can_msg('IS_DAT_DIRA', 2, is_dat_dira)
