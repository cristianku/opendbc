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

    MAX_TORQUE_FACTOR = 100
    MIN_TORQUE_FACTOR = 25

    # Ogni quanto staccare l'EPS per costringerlo a riarmarsi, in SECONDI.
    # Il carcontroller lo converte in frame con DT_CTRL (100 Hz -> 1 frame = 10 ms).
    EPS_REARM_PERIOD = 10.0  # s

    # Per quanto tenere fermo ogni gradino della scaletta 1->2->3->4, in SECONDI.
    # L'EPS campiona a ~10 Hz: sotto ~0.1 s per gradino rischia di saltarne uno.
    # EPS_STATUS_HOLD = 0.15  # s = 3 cicli LKA a 20 Hz = 15 frame
    # EPS_KEEP_STATUS_PERIOD = 0.05
    EPS_ACTIVATE_TAKEOVER_PERIOD = 0.2
    TAKEOVER_MSG_DURATION = 2

    # [CLAUDE eps-closed-loop] - START
    # Quanto aspettare la conferma dell'EPS su un gradino prima di rigenerare il fronte.
    # Misurato su route 0000002a--baede4ffa4: quando l'EPS accetta, ricopia il gradino
    # in 50-110 ms (IS_DAT_DIRA arriva ogni 100 ms). 0.3 s = 3 frame EPS di margine.
    EPS_ACK_TIMEOUT = 0.3  # s
    # [CLAUDE eps-closed-loop] - END

    # [CLAUDE resume-acc-anticipato] - START
    # Sotto quale velocita' iniziare a mandare il finto tasto resume, in m/s.
    # L'ACC di serie della 3008 resta agganciato finche' c'e' un filo di movimento
    # e molla allo zero esatto; da fermo non si riattiva piu' sotto i 30 km/h.
    # Quindi l'impulso deve partire mentre si striscia ancora, non a fermo.
    # 0.56 m/s = 2 km/h. Alzare se il messaggio arriva ancora troppo tardi.
    RESUME_ACC_SPEED = 0.56  # m/s
    # [CLAUDE resume-acc-anticipato] - END

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
  DISABLE_SPEED = 54    # kph
  ENABLE_SPEED = 54     # kph


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
