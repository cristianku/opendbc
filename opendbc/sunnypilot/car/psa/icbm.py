from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.psa.psacan import set_speed
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import (
  IntelligentCruiseButtonManagementInterfaceBase,
)

MAX_TARGET_SPEED_DIFFERENCE_KPH = 20.0
MAX_SET_SPEED_STEP_KPH = 2

ICBMState = (
  structs.IntelligentCruiseButtonManagement
  .IntelligentCruiseButtonManagementState
)


class IntelligentCruiseButtonManagementInterface(
    IntelligentCruiseButtonManagementInterfaceBase):

  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.commanded_speed_kph = None

  def update(self, CC, CC_SP, CS, packer) -> list[CanData]:
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    can_sends = []

    if (
      self.ICBM.state not in (ICBMState.decreasing, ICBMState.holding)
      or not CC.enabled
      or not CS.out.cruiseState.enabled
      or CS.out.vEgo <= 1.0
    ):
      self.commanded_speed_kph = None
      return can_sends

    # Keep target, comparison and transmitted 0x452 in the real CAN/controller
    # domain. The dashboard's own display offset is not published by the port.
    target_speed_kph = round(self.ICBM.vTarget)
    stock_speed_kph = round(CS.out.cruiseState.speed * CV.MS_TO_KPH)

    # The stock 0x452 always remains the source of truth. Sunny may temporarily
    # lower it, but never raises or owns the driver's selected set speed.
    if (
      not 0 < target_speed_kph < 255
      or not 0 < stock_speed_kph < 255
      or target_speed_kph >= stock_speed_kph
    ):
      self.commanded_speed_kph = None
      return can_sends

    # Reject a large discrepancy when starting a command. This catches stale
    # planner targets (the observed 60 -> 29 desynchronization) while still
    # allowing an already active, progressively changing curve target.
    if (
      self.commanded_speed_kph is None
      and stock_speed_kph - target_speed_kph > MAX_TARGET_SPEED_DIFFERENCE_KPH
    ):
      return can_sends

    previous_speed_kph = (
      stock_speed_kph
      if self.commanded_speed_kph is None
      else self.commanded_speed_kph
    )
    self.commanded_speed_kph = max(
      target_speed_kph,
      previous_speed_kph - MAX_SET_SPEED_STEP_KPH,
    )
    self.commanded_speed_kph = min(
      self.commanded_speed_kph,
      previous_speed_kph + MAX_SET_SPEED_STEP_KPH,
    )

    if 0 < self.commanded_speed_kph < 255:
      self.last_button_frame = self.frame
      can_sends.append(set_speed(packer, CS.hs2_dat_mdd_cmd_452, self.commanded_speed_kph))

    return can_sends
