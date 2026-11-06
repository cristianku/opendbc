import pytest

from opendbc.can.packer import CANPacker
from opendbc.car import structs
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.values import CAR, PSA_ADAS_BUS

FrontWipers = structs.CarStateSP.FrontWipers


def test_a_car_that_does_not_say_stays_unknown():
  assert structs.CarStateSP().frontWipers == FrontWipers.unknown


@pytest.mark.parametrize('status, state', [(0, FrontWipers.off), (1, FrontWipers.on), (2, FrontWipers.low), (3, FrontWipers.high)])
def test_the_wipers_are_read_from_0x452(status, state):
  cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_3008)
  interface = CarInterface(cp, CarInterface.get_non_essential_params_sp(cp, CAR.PSA_PEUGEOT_3008))
  interface.update([(1, [])])  # the signals are subscribed on the first read
  frame = CANPacker('psa_aee2010_r3').make_can_msg('HS2_DAT_MDD_CMD_452', PSA_ADAS_BUS, {'FRONT_WIPER_STATUS': status})
  _, ret_sp = interface.update([(10_000_000, [frame])])
  assert ret_sp.frontWipers == state
