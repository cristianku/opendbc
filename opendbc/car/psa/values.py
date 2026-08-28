from dataclasses import dataclass, field

from opendbc.car.structs import CarParams
from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, uds

Ecu = CarParams.Ecu

PSA_ADAS_BUS = 1


class CarControllerParams:
  # STEER_MAX = 250  # Maximum steering torque command that can be applied (unitless scaling factor)
  # # STEER_MAX_LOOKUP = [speed_breakpoints], [torque_values]  # Optional dynamic torque map by vehicle speed
  # STEER_STEP = 5  # Control update frequency (every n frames) – 1 = update at each control loop (100 Hz)
  # STEER_DELTA_UP = 8  # Maximum allowed torque increase per control frame (prevents sudden jumps)
  # STEER_DELTA_DOWN = 38  # Maximum allowed torque decrease per control frame (can be faster for quick release)
  # STEER_DRIVER_MULTIPLIER = 1  # Global weight of driver influence on torque limits (1 = standard sensitivity)
  # STEER_DRIVER_FACTOR = 1  # How strongly driver torque reduces assist torque (higher = more sensitive to driver)
  # STEER_DRIVER_ALLOWANCE = 50  # Deadband (in Nm*10) where driver input does not affect steering assist (prevents interference)
  # MAX_TORQUE_FACTOR = 100
  # MIN_TORQUE_FACTOR = 15

    # Steering torque limits and dynamics for the EPS controller
    STEER_MAX = 150  # Maximum steering torque command that can be applied (unitless scaling factor)
    # STEER_MAX_LOOKUP = [speed_breakpoints], [torque_values]  # Optional dynamic torque map by vehicle speed

    STEER_STEP = 5  # Control update frequency (every n frames) – 1 = update at each control loop (100 Hz)

    STEER_DELTA_UP = 8  # Maximum allowed torque increase per control frame (prevents sudden jumps)
    STEER_DELTA_DOWN = 38  # Maximum allowed torque decrease per control frame (can be faster for quick release)

    STEER_DRIVER_MULTIPLIER = 1  # Global weight of driver influence on torque limits (1 = standard sensitivity)
    STEER_DRIVER_FACTOR = 1  # How strongly driver torque reduces assist torque (higher = more sensitive to driver)
    STEER_DRIVER_ALLOWANCE = 50  # Deadband (in Nm*10) where driver input does not affect steering assist (prevents interference)

    MAX_TORQUE_FACTOR = 100
    MIN_TORQUE_FACTOR = 25

    # [eps curve] - START
    EPS_REARM_PERIOD = 12.0  # s
    EPS_REARM_PERIOD_C4_SPACETOURER = 20.0  # s
    EPS_REARM_EARLIEST_PERIOD = 3.0  # s
    # Keep curve prediction independent from the platform-specific hard deadline: after the
    # 3 s cooldown, use the remaining part of the original 8 s EPS window.
    EPS_REARM_CURVE_LOOKAHEAD = 8.0 - EPS_REARM_EARLIEST_PERIOD  # 5 s
    EPS_REARM_STRAIGHT_LAT_ACCEL = 0.3  # m/s^2
    EPS_REARM_CURVE_LAT_ACCEL = 0.5  # m/s^2
    EPS_TAKEOVER_WARNING_PERIOD = 2.0  # s before the forced EPS rearm
    EPS_TAKEOVER_MODEL_MAX_TIME_GAP = 0.5  # s around the rearm deadline
    # [eps curve] - END

    # Maximum and minimum time allowed for the EPS to reactivate before asking
    # the driver to take over. Lateral acceleration moves the timeout between
    # these bounds, so speed and curvature increase urgency without ever making
    # the takeover request immediate.
    EPS_ACTIVATE_TAKEOVER_MAX_PERIOD = 0.7  # s
    EPS_ACTIVATE_TAKEOVER_MIN_PERIOD = 0.2  # s
    EPS_ACTIVATE_TAKEOVER_FULL_LAT_ACCEL = 1.0  # m/s^2
    TAKEOVER_MSG_DURATION = 2

    EPS_ACK_TIMEOUT = 0.5  # s
    RESUME_ACC_SPEED = 0.56  # m/s

    def __init__(self, CP):
      if CP.carFingerprint == CAR.PSA_CITROEN_C4_SPACETOURER:
        self.EPS_REARM_PERIOD = self.EPS_REARM_PERIOD_C4_SPACETOURER


@dataclass
class PSACarDocs(CarDocs):
  package: str = "Adaptive Cruise Control (ACC) & Lane Assist"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.psa_a]))


@dataclass
class PSAPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'psa_aee2010_r3',
  })


class CAR(Platforms):
  PSA_PEUGEOT_208 = PSAPlatformConfig(
    [PSACarDocs("Peugeot 208 2019-25")],
    CarSpecs(mass=1530, wheelbase=2.73, steerRatio=17.6), # TODO: these are set to live learned Berlingo values
  )
  PSA_PEUGEOT_508 = PSAPlatformConfig(
    [PSACarDocs("Peugeot 508 2019-23")],
    CarSpecs(mass=1720, wheelbase=2.79, steerRatio=17.6), # TODO: set steerRatio
  )
  PSA_PEUGEOT_3008 = PSAPlatformConfig(
    [PSACarDocs("PEUGEOT 3008 2016-29")],
    # https://www.auto-data.net/en/peugeot-3008-ii-phase-i-2016-1.6-puretech-180hp-automatic-s-s-34446#google_vignette
    CarSpecs(mass=1577, wheelbase=2.675, steerRatio=17.69, tireStiffnessFactor=0.996044),
  )
  PSA_CITROEN_C4_SPACETOURER = PSAPlatformConfig(
    [PSACarDocs("CITROEN C4 SPACETOURER 2018-22")],
    # https://www.auto-data.net/en/citroen-c4-spacetourer-phase-i-2018-2.0-bluehdi-163hp-automatic-34644
    CarSpecs(mass=1517, wheelbase=2.785, steerRatio=17.69, tireStiffnessFactor=0.996044),
  )


PSA_DIAG_REQ  = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, 0x01])
PSA_DIAG_RESP = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL + 0x40, 0x01])

PSA_SERIAL_REQ = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER,  0xF1, 0x8C])
PSA_SERIAL_RESP = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40, 0xF1, 0x8C])

PSA_VERSION_REQ  = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER, 0xF0, 0xFE])
PSA_VERSION_RESP = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40, 0xF0, 0xFE])

PSA_RX_OFFSET = -0x20

class LKAS_LIMITS:
  # Peugeot 3008
  # STEER_THRESHOLD: torque (deci-Nm) to detect driver input (steeringPressed)
  # DISABLE/ENABLE_SPEED: LKA hysteresis in km/h
  DISABLE_SPEED = 52    # kph
  ENABLE_SPEED = 51     # kph


FW_QUERY_CONFIG = FwQueryConfig(
  fw_version_regex=br"(?:[A-Z0-9 ]{13,20}|[\x00-\xff]{24})",
  requests=[request for bus in (0, 1, 2) for request in [
    Request(
      [PSA_DIAG_REQ, PSA_SERIAL_REQ],
      [PSA_DIAG_RESP, PSA_SERIAL_RESP],
      rx_offset=PSA_RX_OFFSET,
      bus=bus,
      obd_multiplexing=False,
    ),
    Request(
      [PSA_DIAG_REQ, PSA_VERSION_REQ],
      [PSA_DIAG_RESP, PSA_VERSION_RESP],
      rx_offset=PSA_RX_OFFSET,
      bus=bus,
      obd_multiplexing=False,
    ),
  ]],
  extra_ecus=[
    (Ecu.fwdRadar, 0x6B6, None),
    (Ecu.eps, 0x6B5, None),
    (Ecu.hybrid, 0x6A6, None),
    (Ecu.electricBrakeBooster, 0x5D0, None),
  ],
)

DBC = CAR.create_dbc_map()
