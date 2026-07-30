from opendbc.can.dbc import DBC
from opendbc.can.packer import CANPacker
from opendbc.can.parser import get_raw_value
from opendbc.car import structs
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.values import CAR
from opendbc.sunnypilot.car.psa.icbm import IntelligentCruiseButtonManagementInterface


def create_icbm_test_state(target_kph, actual_kph, stock_setpoint_kph):
  cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_3008)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.PSA_PEUGEOT_3008)
  icbm_interface = IntelligentCruiseButtonManagementInterface(cp, cp_sp)
  packer = CANPacker("psa_aee2010_r3")

  cc = structs.CarControl(enabled=True)
  cc.hudControl.speedVisible = True
  cc_sp = structs.CarControlSP()
  cc_sp.intelligentCruiseButtonManagement.state = structs.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState.holding
  cc_sp.intelligentCruiseButtonManagement.vTarget = target_kph

  cs = type("CarState", (), {})()
  cs.out = structs.CarState(vEgo=actual_kph / 3.6)
  cs.out.cruiseState.enabled = True
  cs.out.cruiseState.speed = stock_setpoint_kph / 3.6
  cs.hs2_dat_mdd_cmd_452 = {"COUNTER": 7, "SPEED_SETPOINT": stock_setpoint_kph}

  return icbm_interface, cc, cc_sp, cs, packer


def test_icbm_metric_target_is_not_converted_twice():
  icbm_interface, cc, cc_sp, cs, packer = create_icbm_test_state(
    target_kph=41,
    actual_kph=44,
    stock_setpoint_kph=45,
  )

  can_sends = icbm_interface.update(cc, cc_sp, cs, packer)
  assert len(can_sends) == 1
  _, dat, _ = can_sends[0]
  msg_def = DBC("psa_aee2010_r3").addr_to_msg[0x452]
  assert get_raw_value(dat, msg_def.sigs["SPEED_SETPOINT"]) == 41


def test_icbm_rejects_target_far_from_actual_speed():
  icbm_interface, cc, cc_sp, cs, packer = create_icbm_test_state(
    target_kph=30,
    actual_kph=50,
    stock_setpoint_kph=60,
  )

  assert icbm_interface.update(cc, cc_sp, cs, packer) == []


def test_icbm_never_increases_stock_setpoint():
  icbm_interface, cc, cc_sp, cs, packer = create_icbm_test_state(
    target_kph=54,
    actual_kph=46,
    stock_setpoint_kph=45,
  )

  assert icbm_interface.update(cc, cc_sp, cs, packer) == []
