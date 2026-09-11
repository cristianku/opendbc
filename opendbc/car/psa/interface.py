from opendbc.car import structs, get_safety_config
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.psa.carcontroller import CarController
from opendbc.car.psa.carstate import CarState
from opendbc.car.psa.values import CAR, LKAS_LIMITS
# [psa longitudinal] - START
from opendbc.car.psa.values import PSA_LONG_CONTROL
# [psa longitudinal] - END

class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  def update(self, can_packets):
    if self.CC.neutral_radar is not None:
      can_packets = self.CC.neutral_radar.process_can(can_packets)
    # [psa longitudinal] - START
    ret, ret_sp = super().update(can_packets)
    if self.CC.longitudinal_profile:
      radar = self.CC.neutral_radar
      if not self.CC.longitudinal_enabled or not radar.active:
        ret.cruiseState.available = False
        ret.cruiseState.enabled = False
      ret.accFaulted = ret.accFaulted or radar.stop_reason is not None
    return ret, ret_sp
    # [psa longitudinal] - END

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = 'psa'

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.psa)]

    #
    ret.dashcamOnly = True

    if candidate in (CAR.PSA_PEUGEOT_3008,CAR.PSA_CITROEN_C4_SPACETOURER):
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      ret.steerControlType = structs.CarParams.SteerControlType.torque
      ret.minSteerSpeed = LKAS_LIMITS.DISABLE_SPEED * CV.KPH_TO_MS
      # Measured from route 00000029--0f498d7077 with FIXED_TORQUE_FACTOR: torque request ->
      # steering rate response lag ~140ms. The old 0.376803 was tuned on the variable
      # torque-factor plant, whose ramp made the EPS look ~4x slower than it is.
      ret.steerActuatorDelay = 0.15
      ret.steerLimitTimer = 0.1
      ret.steerAtStandstill = False
      ret.enableBsm = True
    else:
      ret.steerAtStandstill = True
      ret.steerLimitTimer = 0.1
      ret.steerControlType = structs.CarParams.SteerControlType.angle

    ret.radarUnavailable = True

    # [psa longitudinal] - START
    ret.alphaLongitudinalAvailable = candidate == CAR.PSA_PEUGEOT_3008
    ret.openpilotLongitudinalControl = ret.alphaLongitudinalAvailable and alpha_long
    if ret.openpilotLongitudinalControl:
      ret.dashcamOnly = False
      ret.safetyConfigs[0].safetyParam |= PSA_LONG_CONTROL
    # [psa longitudinal] - END

    return ret

  @staticmethod
  def _get_params_sp(stock_cp, ret, candidate, fingerprint, car_fw,
                    alpha_long, is_release_sp, docs):
    if candidate in (
      CAR.PSA_PEUGEOT_3008,
      CAR.PSA_CITROEN_C4_SPACETOURER,
    ):
      ret.intelligentCruiseButtonManagementAvailable = False
      ret.pcmCruiseSpeed = True

    return ret
