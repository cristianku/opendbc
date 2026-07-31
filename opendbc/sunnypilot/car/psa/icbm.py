from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.psa.psacan import set_speed
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import (
  IntelligentCruiseButtonManagementInterfaceBase,
)

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
      not CC.enabled
      or not CS.out.cruiseState.enabled
    ):
      self.commanded_speed_kph = None
      return can_sends

    # Keep target, comparison and transmitted 0x452 in the real CAN/controller
    # domain. The dashboard's own display offset is not published by the port.
    target_speed_kph = round(self.ICBM.vTarget)
    stock_speed_kph = round(CS.out.cruiseState.speed * CV.MS_TO_KPH)

    if not 0 < stock_speed_kph < 255:
      self.commanded_speed_kph = None
      return can_sends

    # Transmit 0x452 continuously while openpilot and stock cruise are enabled.
    # When ICBM is not actively controlling a lower target, forward the stock
    # setpoint instead of leaving gaps where only the original ECU frame wins.
    desired_speed_kph = stock_speed_kph
    icbm_controls_speed = self.ICBM.state in (
      ICBMState.increasing,
      ICBMState.decreasing,
      ICBMState.holding,
    )
    if icbm_controls_speed and 0 < target_speed_kph < stock_speed_kph:
      desired_speed_kph = target_speed_kph

    previous_speed_kph = (
      stock_speed_kph
      if self.commanded_speed_kph is None
      else self.commanded_speed_kph
    )
    self.commanded_speed_kph = max(
      previous_speed_kph - MAX_SET_SPEED_STEP_KPH,
      min(
        previous_speed_kph + MAX_SET_SPEED_STEP_KPH,
        desired_speed_kph,
      ),
    )
    self.commanded_speed_kph = min(self.commanded_speed_kph, stock_speed_kph)

    self.last_button_frame = self.frame
    can_sends.append(set_speed(packer, CS.hs2_dat_mdd_cmd_452, self.commanded_speed_kph))

    return can_sends
