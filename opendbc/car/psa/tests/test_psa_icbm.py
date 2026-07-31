from opendbc.can.dbc import DBC
from opendbc.can.packer import CANPacker
from opendbc.can.parser import get_raw_value
from opendbc.car import structs
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.values import CAR
from opendbc.sunnypilot.car.psa.icbm import IntelligentCruiseButtonManagementInterface


def get_setpoint(can_sends):
  assert len(can_sends) == 1
  msg_def = DBC("psa_aee2010_r3").addr_to_msg[0x452]
  return get_raw_value(can_sends[0][1], msg_def.sigs["SPEED_SETPOINT"])


def create_icbm_test_state(target_kph, actual_kph, stock_speed_kph):
  cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_3008)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.PSA_PEUGEOT_3008)
  icbm_interface = IntelligentCruiseButtonManagementInterface(cp, cp_sp)
  packer = CANPacker("psa_aee2010_r3")

  cc = structs.CarControl(enabled=True)
  cc.hudControl.speedVisible = True
  cc_sp = structs.CarControlSP()
  cc_sp.intelligentCruiseButtonManagement.state = structs.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState.decreasing
  cc_sp.intelligentCruiseButtonManagement.vTarget = target_kph

  cs = type("CarState", (), {})()
  cs.out = structs.CarState(vEgo=actual_kph / 3.6)
  cs.out.cruiseState.enabled = True
  cs.out.cruiseState.speed = stock_speed_kph / 3.6
  cs.out.cruiseState.speedCluster = stock_speed_kph / 3.6
  cs.hs2_dat_mdd_cmd_452 = {
    "COUNTER": 7,
    "SPEED_SETPOINT": stock_speed_kph,
  }

  return icbm_interface, cc, cc_sp, cs, packer


def test_icbm_ramps_and_sends_real_can_setpoint():
  icbm_interface, cc, cc_sp, cs, packer = create_icbm_test_state(
    target_kph=41,
    actual_kph=44,
    stock_speed_kph=45,
  )

  first_send = icbm_interface.update(cc, cc_sp, cs, packer)
  can_sends = icbm_interface.update(cc, cc_sp, cs, packer)
  assert get_setpoint(first_send) == 43
  assert get_setpoint(can_sends) == 41


def test_icbm_ramps_large_initial_target_desynchronization_without_tx_gap():
  icbm_interface, cc, cc_sp, cs, packer = create_icbm_test_state(
    target_kph=30,
    actual_kph=50,
    stock_speed_kph=60,
  )

  assert get_setpoint(icbm_interface.update(cc, cc_sp, cs, packer)) == 58
  assert get_setpoint(icbm_interface.update(cc, cc_sp, cs, packer)) == 56


def test_icbm_never_increases_stock_setpoint():
  icbm_interface, cc, cc_sp, cs, packer = create_icbm_test_state(
    target_kph=54,
    actual_kph=46,
    stock_speed_kph=45,
  )

  assert get_setpoint(icbm_interface.update(cc, cc_sp, cs, packer)) == 45


def test_icbm_forwards_stock_setpoint_continuously_when_inactive():
  icbm_interface, cc, cc_sp, cs, packer = create_icbm_test_state(
    target_kph=41,
    actual_kph=44,
    stock_speed_kph=45,
  )
  cc_sp.intelligentCruiseButtonManagement.state = structs.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState.inactive

  assert get_setpoint(icbm_interface.update(cc, cc_sp, cs, packer)) == 45
  assert get_setpoint(icbm_interface.update(cc, cc_sp, cs, packer)) == 45


def test_icbm_ramps_back_to_stock_without_tx_gaps():
  icbm_interface, cc, cc_sp, cs, packer = create_icbm_test_state(
    target_kph=41,
    actual_kph=44,
    stock_speed_kph=45,
  )
  assert get_setpoint(icbm_interface.update(cc, cc_sp, cs, packer)) == 43
  assert get_setpoint(icbm_interface.update(cc, cc_sp, cs, packer)) == 41

  cc_sp.intelligentCruiseButtonManagement.state = structs.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState.inactive
  assert get_setpoint(icbm_interface.update(cc, cc_sp, cs, packer)) == 43
  assert get_setpoint(icbm_interface.update(cc, cc_sp, cs, packer)) == 45
