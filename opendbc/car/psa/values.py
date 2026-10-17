from dataclasses import dataclass, field

from opendbc.car.structs import CarParams
from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, uds

Ecu = CarParams.Ecu

PSA_ADAS_BUS = 1

PSA_LONG_CONTROL = 1  # safetyParam bit selected by alpha_long on Peugeot 3008.


class LongitudinalParams:
  # In: accel + 9.81*sin(pitch) [m/s^2]. Out: Nm. 0..1 from route 49; -1, -0.5, +1.5, +2 provisional.
  POSITIVE_JERK_MAX = 3.5
  TORQUE_FILTER_RC = 0.20  # ~0.47 s to reach 90% of a positive torque step at 100 Hz.
  BRAKE_FILTER_RC = 0.10  # ~0.23 s to reach 90% of a stronger brake request at 100 Hz.
  ACCEL_LOOKUP = (-1.0, -0.5, 0.0, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0)
  TORQUE_LOOKUP = (-400, -300, 179, 301, 424, 547, 670, 800, 1000)
  POTENTIAL_TORQUE_LOOKUP = (-400, -300, 169, 279, 390, 501, 612, 800, 1000)
  BRAKE_ENTER_ACCEL = -0.5  # Provisional entry crossover; not a lower limit on light braking.
  # Brake mode hysteresis on equivalent_accel = accel + 9.81*sin(pitch), m/s^2.
  BRAKE_EXIT_ACCEL = 0.0  # leave the brakes when the engine alone is enough
  BRAKE_EXIT_POSITIVE_ACCEL = 0.2  # or when a real positive request is reachable by torque
  PITCH_FILTER_RC = 0.5  # s, as Toyota
  # accel -> MDD_DESIRED_DECELERATION, +10%: routes 118/119 settle ~10% short of the request.
  BRAKE_ACCEL_BP = (
    -2.0, -1.5, -1.0, -0.75,
    -0.5, -0.25, 0.0
  )
  BRAKE_ACCEL_V = (
    -2.0, -1.65, -1.1, -0.825,
    -0.55, -0.275, 0.0
  )
  BRAKE_MIN_ACCEL = -2.0  # Provisional service-brake limit, m/s^2; keep in sync with PSA safety.
  MIN_TIME_GMP_EXPERIMENTAL = 6.2  # Does not reproduce the observed even/odd sequence.
  INACTIVE_ACCEL = 2.05
  INACTIVE_TORQUE = -4000


class CarControllerParams:

    # Steering torque limits and dynamics for the EPS controller
    STEER_MAX = 150  # Maximum steering torque command that can be applied (unitless scaling factor)

    STEER_STEP = 1  # 100 Hz, every control frame
    STEER_DELTA_UP = 2  # per 10 ms frame: 200/s (was 8 per 50 ms = 160/s)
    STEER_DELTA_DOWN = 8  # per 10 ms frame: 800/s (was 38 per 50 ms = 760/s)
    EPS_LADDER_STEP = 5  # activation ladder and rearm checks stay at 20 Hz

    # CarState.steeringTorque units (3008: raw DRIVER_TORQUE * 3).
    STEER_DRIVER_MULTIPLIER = 1
    STEER_DRIVER_ALLOWANCE = 50
    STEER_DRIVER_FACTOR = 1  # How strongly driver torque reduces assist torque (higher = more sensitive to driver)

    MAX_TORQUE_FACTOR = 100
    MIN_TORQUE_FACTOR = 25

    EPS_REARM_PERIOD = 12.0  # s
    EPS_REARM_PERIOD_C4_SPACETOURER = 12.0  # s
    EPS_REARM_EARLIEST_PERIOD = 3.0  # s
    # After the 3 s cooldown, look ahead over the rest of the 8 s EPS window.
    EPS_REARM_CURVE_LOOKAHEAD = 8.0 - EPS_REARM_EARLIEST_PERIOD  # 5 s
    EPS_REARM_STRAIGHT_LAT_ACCEL = 0.3  # m/s^2
    EPS_REARM_CURVE_LAT_ACCEL = 0.5  # m/s^2
    EPS_TAKEOVER_WARNING_PERIOD = 2.0  # s before the forced EPS rearm
    EPS_TAKEOVER_MODEL_MAX_TIME_GAP = 0.5  # s around the rearm deadline

    # One smooth burst per EPS cycle: sin^2 envelope * sine, sign alternated.
    EPS_DITHER_AMPLITUDE = 0  # peak, effective torque units; 0 = off (6 when enabled)
    EPS_DITHER_FREQUENCY = 2.5  # Hz
    EPS_DITHER_WAVES = 3  # whole waves -> 1.2 s, zero mean for the complete nominal burst
    EPS_DITHER_START_MARGIN = 2.0  # s after EPS activation
    EPS_DITHER_END_MARGIN = 3.0  # s before the rearm

    # During EPS reactivation, request immediate driver takeover only on a sufficiently large curve.
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
  # LKA speed hysteresis, km/h
  DISABLE_SPEED = 51    # kph
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
