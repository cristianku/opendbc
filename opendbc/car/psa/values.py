from dataclasses import dataclass, field

from opendbc.car.structs import CarParams
from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, uds

Ecu = CarParams.Ecu


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

    # Increasing STEER_MAX increases resolution (number of torque steps).
    # MAX_TORQUE_FACTOR limits the effective range (percent of STEER_MAX).
    # Example of total available steps:
  #   -------------------------------------------------------------
    #   STEER_MAX | MAX_TORQUE_FACTOR | Effective Range (±R) | Steps (±)
    #   -----------+-------------------+---------------------+------------
    #      100     |       100         |        ±100         |   ±100
    #      200     |        50         |        ±100         |   ±200
    #      400     |        25         |        ±100         |   ±400
    #   -------------------------------------------------------------
    # Higher STEER_MAX + lower torque factor = finer granularity with same peak torque.
    MAX_TORQUE_FACTOR = 100
    MIN_TORQUE_FACTOR = 25

    # [CLAUDE eps-rearm] - START
    # L'EPS smette di assistere dopo un po' di attivazione continua (visto in
    # autostrada, rettilineo lungo = richiesta di coppia bassa e costante).
    # Farlo uscire da ACTIVE per una frazione di secondo lo fa ri-armare.
    # Solo in rettilineo e a velocita' autostradale: in curva perdere l'assist
    # per 100 ms si sentirebbe.
    # ATTENZIONE: questo e' l'enum del messaggio TX LANE_KEEP_ASSIST (1010):
    #   0 UNAVAILABLE, 1 UNSELECTED, 2 SELECTED, 3 AUTHORIZED, 4 ACTIVE, 5 DEFECT
    # NON quello RX di IS_DAT_DIRA.EPS_STATE_LKA (1173), che e' shiftato:
    #   0 Unauthorised, 1 Authorised, 2 Available, 3 Active, 4 Defect
    # STATUS mandato durante l'impulso: 2 SELECTED e' il gradino da cui _activate_eps
    # risale sempre (quindi provato in auto). Se l'EPS non si ri-arma, provare 1
    # UNSELECTED, poi 0 UNAVAILABLE (ultima spiaggia: sa di "sistema guasto", DTC).
    EPS_REARM_STATUS = 2
    EPS_REARM_PERIOD = 5.0            # s di EPS attivo prima di un impulso
    EPS_REARM_LENGTH = 0.10           # s di impulso (2 msg LKA @20Hz)
    EPS_REARM_MAX_CURVATURE = 0.001   # 1/m sulla curvatura richiesta -> raggio > 1000 m
    EPS_REARM_MAX_ANGLE = 10.0        # deg volante, guardia per non scattare in curva
    # EPS_REARM_MIN_SPEED = 22.0      # m/s (~80 km/h), solo autostrada
    EPS_REARM_MIN_SPEED_KPH = 54.0    # km/h, solo autostrada (convertito in m/s nel CarController)
    # [CLAUDE eps-rearm] - END

    def __init__(self, CP):
      pass


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
  DISABLE_SPEED = 50    # kph
  ENABLE_SPEED = 50     # kph

FW_QUERY_CONFIG = FwQueryConfig(
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
  ]]
)

DBC = CAR.create_dbc_map()
