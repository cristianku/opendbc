from opendbc.car import structs, DT_CTRL
from opendbc.car.can_definitions import CanData
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.psa.psacan import set_speed
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import (
  IntelligentCruiseButtonManagementInterfaceBase,
)

MAX_TARGET_SPEED_DIFFERENCE_KPH = 20.0

ICBMState = (
  structs.IntelligentCruiseButtonManagement
  .IntelligentCruiseButtonManagementState
)


class IntelligentCruiseButtonManagementInterface(
    IntelligentCruiseButtonManagementInterfaceBase):

  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)

  def update(self, CC, CC_SP, CS, packer, last_button_frame) -> list[CanData]:
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.last_button_frame = last_button_frame
    can_sends = []

    if (
      self.ICBM.state == ICBMState.inactive
      or not CC.enabled
      or not CC.hudControl.speedVisible
      or not CS.out.cruiseState.enabled
      or CS.out.vEgo <= 1.0
    ):
      return []

    # Sunny publishes ICBM.vTarget in the selected cluster unit. The PSA
    # integration currently supports the metric setting, so this is already kph.
    set_speed_kph = round(self.ICBM.vTarget)
    # actual_speed_kph = CS.out.vEgo * CV.MS_TO_KPH
    # stock_set_speed_kph = round(CS.out.cruiseState.speed * CV.MS_TO_KPH)

    # This direct PSA backend is only allowed to reduce the driver's stock
    # setpoint. Ignore large target jumps and let the untouched stock 0x452 take
    # over instead of injecting an implausible command.
    # if (
    #   not 0 < set_speed_kph < 255
    #   # or set_speed_kph >= stock_set_speed_kph
    #   or abs(set_speed_kph - actual_speed_kph) > MAX_TARGET_SPEED_DIFFERENCE_KPH
    # ):
    #   return []

    if (self.frame - self.last_button_frame) * DT_CTRL > 0.2:
      if 0 < set_speed_kph < 255:
        self.last_button_frame = self.frame
        can_sends.append([set_speed(packer,CS.hs2_dat_mdd_cmd_452, set_speed_kph)])

    return can_sends
