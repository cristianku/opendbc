from opendbc.can.dbc import DBC
from opendbc.can.packer import CANPacker
from opendbc.can.parser import get_raw_value
from opendbc.car import structs
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.values import CAR
from opendbc.sunnypilot.car.psa.icbm import IntelligentCruiseButtonManagementInterface


def test_icbm_metric_target_is_not_converted_twice():
  cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_3008)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.PSA_PEUGEOT_3008)
  icbm_interface = IntelligentCruiseButtonManagementInterface(cp, cp_sp)
  packer = CANPacker("psa_aee2010_r3")

  cc = structs.CarControl(enabled=True)
  cc.hudControl.speedVisible = True
  cc_sp = structs.CarControlSP()
  cc_sp.intelligentCruiseButtonManagement.state = structs.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState.holding
  cc_sp.intelligentCruiseButtonManagement.vTarget = 28

  cs = type("CarState", (), {})()
  cs.out = structs.CarState(vEgo=50 / 3.6)
  cs.out.cruiseState.enabled = True
  cs.hs2_dat_mdd_cmd_452 = {"COUNTER": 7, "SPEED_SETPOINT": 28}

  can_sends = icbm_interface.update(cc, cc_sp, cs, packer)

  assert len(can_sends) == 1
  _, dat, _ = can_sends[0]
  msg_def = DBC("psa_aee2010_r3").addr_to_msg[0x452]
  assert get_raw_value(dat, msg_def.sigs["SPEED_SETPOINT"]) == 28
