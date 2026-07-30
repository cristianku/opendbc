from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.psa.psacan import set_speed
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import (
  IntelligentCruiseButtonManagementInterfaceBase,
)

ICBMState = (
  structs.IntelligentCruiseButtonManagement
  .IntelligentCruiseButtonManagementState
)


class IntelligentCruiseButtonManagementInterface(
    IntelligentCruiseButtonManagementInterfaceBase):

  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)

  def update(self, CC, CC_SP, CS, packer) -> list[CanData]:
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement

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

    if not 0 < set_speed_kph < 255:
      return []

    return [
      set_speed(
        packer,
        CS.hs2_dat_mdd_cmd_452,
        set_speed_kph,
      )
    ]
