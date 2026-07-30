from opendbc.car import structs, get_safety_config
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.psa.carcontroller import CarController
from opendbc.car.psa.carstate import CarState
from opendbc.car.psa.values import CAR, LKAS_LIMITS

class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = 'psa'

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.psa)]

    #
    ret.dashcamOnly = False

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

    ret.alphaLongitudinalAvailable = False
    # ret.openpilotLongitudinalControl = alpha_long
    ret.startingState = True
    ret.startAccel = 1.0

    return ret

  @staticmethod
  def _get_params_sp(stock_cp, ret, candidate, fingerprint, car_fw,
                    alpha_long, is_release_sp, docs):
    if candidate in (
      CAR.PSA_PEUGEOT_3008,
      CAR.PSA_CITROEN_C4_SPACETOURER,
    ):
      ret.intelligentCruiseButtonManagementAvailable = True
      ret.pcmCruiseSpeed = False

    return ret