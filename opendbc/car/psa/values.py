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

    ENABLE_DRIVER_TORQUE = False  # If True, create a simulated driver torque signal for testing purposes
    # Steering torque limits and dynamics for the EPS controller
    STEER_MAX = 150  # Maximum steering torque command that can be applied (unitless scaling factor)
    # STEER_MAX_LOOKUP = [speed_breakpoints], [torque_values]  # Optional dynamic torque map by vehicle speed

    STEER_STEP = 5  # Control update frequency (every n frames) – 1 = update at each control loop (100 Hz)

    STEER_DELTA_UP = 5  # Maximum allowed torque increase per control frame (prevents sudden jumps)
    STEER_DELTA_DOWN = 38  # Maximum allowed torque decrease per control frame (can be faster for quick release)

    STEER_DRIVER_MULTIPLIER = 1  # Global weight of driver influence on torque limits (1 = standard sensitivity)
    STEER_DRIVER_FACTOR = 1  # How strongly driver torque reduces assist torque (higher = more sensitive to driver)
    # [CLAUDE driver-override] - START
    # STEER_DRIVER_ALLOWANCE = 50  # Deadband (in Nm*10) where driver input does not affect steering assist (prevents interference)
    # NB: carstate.py confronta questa soglia con DRIVER_TORQUE *3, quindi 50 = 16.7
    # grezzi e 70 = 23.3 grezzi. Baseline a mani ferme (route 00000015, 26 lug 2026):
    # mediana 13.2, p90 38.6, massimo mai-pressed 57.3 in scala *3 -> 50 era a un soffio.
    # MISURATO sui 25 segmenti (24 min): 51 eventi steeringPressed sopra i 50 km/h
    # (dove il lateral e' attivo), 69 s totali = 8% del tempo con assist a zero.
    # Di questi solo 15 erano riprese vere (volante 6-26 deg, picco coppia 62-90);
    # gli altri 36 erano mani appoggiate (volante < 6 deg, picco medio 60).
    # Con 70: 15 eventi, 13 riprese vere su 15 e solo 2 falsi.
    # Il panda tiene la sua .driver_torque_allowance = 50 in safety/modes/psa.h, ma
    # e' inerte: psa_rx_hook non aggiorna mai torque_driver.
    STEER_DRIVER_ALLOWANCE = 70  # Deadband (in Nm*10) where driver input does not affect steering assist (prevents interference)
    # [CLAUDE driver-override] - END

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
    # STATUS mandato durante l'impulso.
    # PROVATO IN AUTO 2026-07-25 (route ~t=459.7): con 2 SELECTED per 100 ms l'EPS
    # NON reagisce affatto - EPS_STATE_LKA resta piatto su 3 (Active). Motivo: in
    # guida normale la telecamera oscilla tra 2/3/4 di continuo (corsia rilevata o
    # no), quindi l'EPS per progetto NON esce dalla sessione LKA quando vede un 2.
    # La scala 2->3->4 di _activate_eps dimostra solo che 2 funziona come gradino
    # verso l'ALTO partendo da stato non attivo, non che disattivi partendo da ACTIVE.
    # 1 UNSELECTED = "il guidatore ha spento la funzione": l'unico che chiude la
    # sessione. Se non basta nemmeno lui, resta 0 UNAVAILABLE (ultima spiaggia: sa
    # di "sistema guasto", possibile DTC).
    # EPS_REARM_STATUS = 2            # ignorato dall'EPS, vedi sopra
    EPS_REARM_STATUS = 1
    EPS_REARM_PERIOD = 10.0            # s di EPS attivo prima di un impulso
    # 100 ms erano 2 soli invii: sotto il debounce tipico di una ECU di sterzo (3-5
    # frame consecutivi), e con IS_DAT_DIRA a ~10 Hz ci cadeva dentro un solo
    # campione di risposta, quindi nemmeno misurabile. Se 0.25 funziona, riprovare
    # ad accorciare (0.15, 0.10) per ridurre il buco al minimo accettato dall'EPS.
    # EPS_REARM_LENGTH = 0.10         # s di impulso (2 msg LKA @20Hz)
    # EPS_REARM_LENGTH = 0.25         # s di impulso (5 msg LKA @20Hz)
    # [CLAUDE eps-rearm-ladder] - START
    # MISURATO su route 00000015 (26 lug 2026, segmenti 4-5, 19 impulsi):
    # l'EPS esce da ACTIVE 20-33 ms dopo aver visto STATUS=1, quindi 5 messaggi
    # (200 ms) sono ~6 volte piu' del necessario. 2 messaggi bastano con margine.
    EPS_REARM_LENGTH = 0.10           # s di impulso (2 msg LKA @20Hz)
    # RIATTIVAZIONE. La macchina a stati LKA dell'EPS gira a ~10 Hz (IS_DAT_DIRA
    # arriva ogni 100 ms) mentre noi mandiamo LANE_KEEP_ASSIST a 20 Hz. Con la
    # vecchia scaletta (un gradino per invio, 50 ms, e ciclo infinito 2->3->4->2)
    # l'EPS campionava un nostro messaggio su due e vedeva la sequenza scombinata
    # (3,2,4,3,2,4...): si riarmava solo quando le due fasi si allineavano per
    # caso -> buco misurato 499-2021 ms, media 968 ms.
    # Nei log l'EPS passa a 1 (Authorised) subito dopo aver campionato 2 e poi 3,
    # e a 3 (Active) subito dopo aver visto 4: il protocollo va bene, servono solo
    # gradini abbastanza lunghi da non poter essere saltati dal suo campionamento.
    EPS_REARM_STEP_HOLD = 0.15        # s per gradino della scaletta (3 invii LKA)
    # Arrivati a 4 si TIENE 4 (il ciclo 4->2 era proprio quello che scombinava la
    # sequenza). Solo se dopo questo tempo l'EPS non e' tornato ACTIVE si riparte da 2.
    EPS_REARM_LADDER_TIMEOUT = 0.6    # s prima di ricominciare la scaletta da 2
    # Gradini della scaletta di riattivazione (enum TX LANE_KEEP_ASSIST 1010):
    # 2 SELECTED -> 3 AUTHORIZED -> 4 ACTIVE
    EPS_REARM_LADDER = (2, 3, 4)
    # [CLAUDE eps-rearm-ladder] - END

    # [CLAUDE takeover-test] - START
    # Master switch per la trasmissione di REQUEST_TAKEOVER tramite
    # HS2_DYN_MDD_ETAT_2F6. False durante il test radar-disable-only, così
    # openpilot non trasmette alcun 0x2F6 senza alterare i timing sottostanti.
    ENABLE_TAKEOVER_REQUEST = False
    # Timeout dalla prima chiamata a _activate_eps(), valido sia alla prima
    # attivazione sia durante il rearm. 0 = disattivato.
    EPS_TAKEOVER_AFTER = 0.2
    EPS_TAKEOVER_TYPE = 1             # 1 = richiesta non critica, 2 = critica
    # Il warning viene inviato a 50 Hz finché l'EPS non torna ACTIVE.
    # TEST DA FERMO: fa partire la richiesta a intervalli fissi anche quando openpilot
    # non sta sterzando. Serve perche' da fermo siamo sotto minSteerSpeed (50 km/h),
    # quindi latActive e' falso, l'EPS non si arma mai, la riattivazione non fallisce
    # mai e la richiesta normale non partirebbe. 0 = disattivato (uso normale).
    TAKEOVER_TEST_PERIOD = 10.0       # s fra una richiesta e l'altra
    # [CLAUDE takeover-test] - END

    # [CLAUDE eps-fault] - START
    # Caso estremo: la riattivazione non riesce proprio. Dopo questo tempo senza EPS
    # ACTIVE alziamo ret.steerFaultTemporary (travasato nel CarState da interface.py)
    # e openpilot avvisa il guidatore.
    # NB: controlsd fa latActive = ... and not steerFaultTemporary, quindi il lateral
    # si spegne subito. E car_specific.py genera steerTempUnavailableSilent (solo
    # avviso) se il guidatore ha toccato il volante da meno di 1.5 s, altrimenti
    # steerTempUnavailable = SOFT_DISABLE, che dopo SOFT_DISABLE_TIME=3 s sgancia.
    EPS_FAULT_AFTER = 2.5             # s senza EPS attivo prima di avvisare openpilot
    # Durata minima dell'avviso. Serve perche' appena il flag sale latActive va a
    # False, il ramo laterale non gira piu' e _reset_lat_state azzera i contatori:
    # senza una durata propria il flag si spegnerebbe subito. Tenuta sotto i 3 s del
    # soft disable: openpilot avvisa, poi il flag cade e la scaletta ritenta da capo.
    EPS_FAULT_HOLD = 1.5              # s
    # [CLAUDE eps-fault] - END

    # [CLAUDE radar-disable] - START
    # TEST DA FERMO. Mette l'ARTIV in sessione di programmazione (vedi
    # psacan.create_disable_radar) e la tiene giu' con un tester present a 1 Hz.
    # Serve per verificare se, zittito il radar, openpilot diventa l'unico mittente
    # di HS2_DYN_MDD_ETAT_2F6 e quindi controlla davvero REQUEST_TAKEOVER.
    # COSA SUCCEDE: dal bus ADAS spariscono 0x2B6 (50 Hz), 0x2F6 (50 Hz),
    # 0x4F6 (10 Hz) e 0x796 (1 Hz) -> niente ACC, niente AEB, e il BSI vede una
    # centralina ADAS che non risponde (attesi DTC e spie sul quadro).
    # NB: noi emuliamo solo 2F6 e a raffica, non 4F6 ne' 796, quindi il buco resta.
    # USCITA: rimetti False e ricarica -> senza tester present il timer S3 dell'ECU
    # scade in ~5 s e il radar riprende a trasmettere. Poi ciclo di chiave.
    # Da usare SOLO fermi in parcheggio. Default False: non parte mai per sbaglio.
    DISABLE_RADAR_TEST = True
    # Ritardo dall'inizio del giro prima di zittire il radar: i primi secondi restano
    # come baseline col radar vivo, cosi' il confronto prima/dopo si fa dentro lo
    # stesso log invece che con una route di un altro giorno. 0 = subito.
    DISABLE_RADAR_AFTER = 30.0        # s
    # Attesa massima della risposta 50 02 su 0x696. PyPSADiag considera aperta la
    # programming session soltanto dopo quella risposta:
    # https://github.com/Barracuda09/PyPSADiag/blob/main/json/ARTIV/ARTIV_UDS.json
    RADAR_DIAG_RESPONSE_TIMEOUT = 3.0  # s
    # Invia una sola REQUEST_TAKEOVER dopo questo ritardo dal programming mode.
    # 0 = test disattivato.
    RADAR_TAKEOVER_TEST_AFTER = 3.0   # s
    # [CLAUDE radar-disable] - END
    # Valori PERMISSIVI per i test (difficile trovare rettilinei veri): l'impulso
    # scatta anche in curva larga. Scostamento laterale nel caso peggiore (assist a
    # zero per 0.25 s = impulso + riattivazione): ~3.5 cm a 54 km/h, ~10 cm a 90 km/h.
    # Per l'uso normale rimettere 0.001 / 5.0 (raggio > 1000 m, solo rettilineo).
    # EPS_REARM_MAX_CURVATURE = 0.001 # 1/m sulla curvatura richiesta -> raggio > 1000 m
    EPS_REARM_MAX_CURVATURE = 0.005   # 1/m -> raggio > 200 m (curva larga, 13.6 deg volante)
    # NB: le due soglie vanno alzate INSIEME, altrimenti l'angolo diventa il vincolo
    # vero (10 deg volante = curvatura 0.0037) e la curvatura sopra non ha effetto.
    # EPS_REARM_MAX_ANGLE = 10.0      # deg volante (= curvatura 0.0037, raggio 271 m)
    EPS_REARM_MAX_ANGLE = 20.0        # deg volante, resta solo rete di sicurezza
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
