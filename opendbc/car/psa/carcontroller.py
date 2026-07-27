from opendbc.can.packer import CANPacker
# [eps-rearm] - START
# from opendbc.car import Bus, structs
# [CLAUDE artiv-nopad] - START
# Riga sostituita (make_tester_present_msg non piu' usato: padda a DLC 8,
# sostituito da psacan.create_tester_present):
# from opendbc.car import Bus, structs, DT_CTRL, make_tester_present_msg
from opendbc.car import Bus, structs, DT_CTRL
# [CLAUDE artiv-nopad] - END
from opendbc.car.common.conversions import Conversions as CV
# [eps-rearm] - END
from opendbc.car.carlog import carlog
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
# from opendbc.car.psa.psacan import create_lka_steering, create_driver_torque, create_steering_hold, create_resume_acc, create_HS2_DYN1_MDD_ETAT_2B6, create_HS2_DYN_MDD_ETAT_2F6
# [takeover-test] - START
# from opendbc.car.psa.psacan import create_lka_steering, create_driver_torque, create_steering_hold, create_resume_acc,  create_HS2_DYN1_MDD_ETAT_2B6, create_HS2_DYN_MDD_ETAT_2F6
# [artiv-diag-probe] - START
# from opendbc.car.psa.psacan import (create_lka_steering, create_driver_torque, create_steering_hold, create_request_takeover)
# [CLAUDE artiv-nopad] - START
# Riga sostituita (aggiunto create_tester_present, versione senza padding):
# from opendbc.car.psa.psacan import (create_lka_steering, create_driver_torque, create_steering_hold, create_request_takeover,
#                                     create_disable_radar)
from opendbc.car.psa.psacan import (create_lka_steering, create_driver_torque, create_steering_hold, create_request_takeover,
                                    create_disable_radar, create_tester_present)
# [CLAUDE artiv-nopad] - END
# [artiv-diag-probe] - END
# [takeover-test] - END
from opendbc.car.psa.values import CarControllerParams, CAR
from cereal import messaging
# from numpy import interp

import random

from opendbc.sunnypilot.car.hyundai.tests.test_tuning_controller import CS
# import math

SteerControlType = structs.CarParams.SteerControlType
sm = messaging.SubMaster(['modelV2'], poll='modelV2')

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.first_message = False
    self.apply_torque_last = 0
    self.apply_can_torque_last = 0  # raw CAN torque logged to steeringAngleDeg (debug); init so it always exists
    self.apply_torque_factor = 0
    self.apply_torque = 0
    self.status = 2
    self.send_takeover_req = False
    self.takeover_req_sent = False
    # this is the frame when the latactive is being pressed
    self.lat_activation_frame  = 0
    self.lat_active_last = False
    self.car_fingerprint = CP.carFingerprint
    self.params = CarControllerParams(CP)
    self.artiv_diag_probe_sent = False
    self.artiv_diag_probe_done = False
    self.artiv_diag_probe_sent_frame = 0
    # [radar-disable] - START
    self.radar_disabled = 0   # 0 = ancora da disabilitare; il 10 02 parte una volta a frame 0
    # [radar-disable] - END
    self.bars = 4
    self.steering_hold_counter = 0
    self.next_steering_hold = random.randint(8, 12)  # ~10Hz con jitter ±20%
    self.driver_torque_counter = 0
    self.next_driver_torque = random.randint(500, 800)  # 5–8 s @100 Hz
    # [eps-rearm] - START
    # Contatori in "invii LKA" (uno ogni STEER_STEP frame -> 20 Hz).
    self.eps_rearm_step = DT_CTRL * self.params.STEER_STEP                                        # 0.05 s
    self.eps_rearm_period = int(self.params.EPS_REARM_PERIOD / self.eps_rearm_step)                # 100 invii = 5 s
    self.eps_rearm_len = max(1, int(round(self.params.EPS_REARM_LENGTH / self.eps_rearm_step)))    # 2 invii = 100 ms
    self.eps_rearm_counter = 0   # invii LKA con EPS attivo dall'ultimo impulso
    self.eps_rearm_pulse = 0     # invii LKA rimanenti dell'impulso in corso
    # Soglia in km/h in values.py, convertita in m/s una volta sola qui (vEgo e' in m/s).
    self.eps_rearm_min_speed = self.params.EPS_REARM_MIN_SPEED_KPH * CV.KPH_TO_MS    # 80 km/h = 22.2 m/s
    # [eps-rearm] - END
    # [eps-rearm-ladder] - START
    # Scaletta di riattivazione deterministica: ogni gradino tenuto per piu' invii,
    # cosi' l'EPS (che campiona a ~10 Hz) non puo' saltarne uno. Vedi values.py.
    # Tutto in "invii LKA": 1 invio = STEER_STEP frame = 50 ms.
    self.eps_rearm_hold = max(1, int(round(self.params.EPS_REARM_STEP_HOLD / self.eps_rearm_step)))     # 3 invii = 150 ms
    self.eps_rearm_top_hold = max(1, int(round(self.params.EPS_REARM_TOP_HOLD / self.eps_rearm_step)))  # 6 invii = 300 ms
    self.eps_rearm_ladder_cycle = ((len(self.params.EPS_REARM_LADDER) - 1) * self.eps_rearm_hold
                                   + self.eps_rearm_top_hold)                                          # 12 invii = 600 ms
    self.rearm_tick = 0   # invii dall'inizio della scaletta; gradino e factor si ricavano da qui
    # [eps-rearm-ladder] - END
    # [CLAUDE eps-rearm-blackout] - START
    # Invii per cui, finito l'impulso, eps_active va considerato False comunque:
    # il dato RX e' piu' lento dell'impulso stesso (vedi values.py).
    self.eps_rearm_blackout = max(1, int(round(self.params.EPS_REARM_BLACKOUT / self.eps_rearm_step)))   # 8 invii = 400 ms
    self.eps_rearm_confirm = 0   # invii rimanenti di blackout; 0 = ci si fida di CS.eps_active
    # [CLAUDE eps-rearm-blackout] - END
    # [takeover-test] - START
    # Tempo totale in cui l'EPS non e' tornato ACTIVE, usato dal timeout EPS fault.
    self.eps_rearm_wait = 0
    # test da fermo: richiesta periodica anche senza latActive (0 = disattivato)
    self.takeover_test_frames = int(round(self.params.TAKEOVER_TEST_PERIOD / DT_CTRL))   # 1000 frame = 10 s
    # [takeover-test] - END
    # [artiv-diag-probe] - START
    self.artiv_diag_probe_frame = int(round(self.params.ARTIV_DIAG_PROBE_AFTER / DT_CTRL))
    self.artiv_diag_probe_timeout_frames = int(round(self.params.ARTIV_DIAG_RESPONSE_TIMEOUT / DT_CTRL))
    # [artiv-diag-probe] - END
    # [eps-fault] - START
    self.eps_rearm_failed = False   # letto da interface.py -> ret.steerFaultTemporary
    self.eps_fault_sends = int(round(self.params.EPS_FAULT_AFTER / self.eps_rearm_step))     # 50 invii = 2.5 s
    self.eps_fault_hold_frames = int(round(self.params.EPS_FAULT_HOLD / DT_CTRL))            # 150 frame = 1.5 s
    self.eps_fault_hold_cnt = 0
    # [eps-fault] - END

  def _reset_lat_state(self):
    self.status = 2
    self.apply_torque_factor = 0
    self.send_takeover_req = False
    self.takeover_req_sent = False
    self.lat_activation_frame = 0
    # [eps-rearm] - START
    self.eps_rearm_counter = 0
    self.eps_rearm_pulse = 0
    # [eps-rearm] - END
    # [CLAUDE eps-rearm-blackout] - START
    self.eps_rearm_confirm = 0
    # [CLAUDE eps-rearm-blackout] - END
    # [eps-rearm-ladder] - START
    self._reset_rearm_ladder()
    # [eps-rearm-ladder] - END

  # [eps-rearm-ladder] - START
  def _reset_rearm_ladder(self):
    self.rearm_tick = 0   # prossima partenza dal gradino 2, factor basso
    self.eps_rearm_wait = 0
    self.send_takeover_req = False
    self.takeover_req_sent = False
  # [eps-rearm-ladder] - END

  # [eps-rearm] - START
  def _going_straight(self, CC, CS):
    # Rettilineo + velocita' autostradale + mani a posto: l'unica condizione in cui
    # mollare l'assist per una frazione di secondo e' innocuo.
    return (abs(CC.actuators.curvature) < self.params.EPS_REARM_MAX_CURVATURE and
            abs(CS.out.steeringAngleDeg) < self.params.EPS_REARM_MAX_ANGLE and
            # CS.out.vEgo > self.params.EPS_REARM_MIN_SPEED and   # era in m/s
            CS.out.vEgo > self.eps_rearm_min_speed and
            not CS.out.steeringPressed)
  # [eps-rearm] - END

  # [eps-rearm-ladder] - START
  def _activate_eps(self):
    if self.lat_activation_frame == 0:
      self.lat_activation_frame = self.frame

    # if (self.params.EPS_TAKEOVER_AFTER > 0 and not self.takeover_req_sent and not self.send_takeover_req
    #     and (self.frame - self.lat_activation_frame) * DT_CTRL >= self.params.EPS_TAKEOVER_AFTER):
    #   self.send_takeover_req = True
    self.send_takeover_req = True

    # Scaletta di riattivazione 2 SELECTED -> 3 AUTHORIZED -> 4 ACTIVE. rearm_tick =
    # invii dall'inizio: un gradino ogni eps_rearm_hold invii, il 4 tenuto per
    # eps_rearm_top_hold, poi si riparte da 2 (factor incluso, si ricava dal gradino).
    ladder = self.params.EPS_REARM_LADDER
    if self.rearm_tick >= self.eps_rearm_ladder_cycle:
      self.rearm_tick = 0
    pos = min(self.rearm_tick // self.eps_rearm_hold, len(ladder) - 1)
    self.status = ladder[pos]
    self.apply_torque_factor = min(10 * (pos + 1), self.params.MAX_TORQUE_FACTOR)
    self.rearm_tick += 1
    self.eps_rearm_wait += 1
  # [eps-rearm-ladder] - END

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators
    self.apply_new_torque = 0
    # apply_new_torque = 0
    temp_driverSteeringTorque = 0
    new_torque_scaled = 0
    apply_new_torque_scaled = 0
    if CC.latActive != self.lat_active_last:
      carlog.error(f"PSA_DEBUG latActive={CC.latActive}")
      self.lat_active_last = CC.latActive

    # [eps-fault] - START
    # Il rilascio sta QUI e non nel ramo laterale: appena il flag sale, openpilot
    # porta latActive a False e quel ramo non gira piu' (e _reset_lat_state azzera i
    # contatori), quindi da li' non si spegnerebbe mai.
    if self.eps_fault_hold_cnt > 0:
      self.eps_fault_hold_cnt -= 1
      if self.eps_fault_hold_cnt == 0:
        self.eps_rearm_failed = False   # avviso finito: si ritenta la riattivazione
    # [eps-fault] - END

    # lateral control
    if self.CP.steerControlType == SteerControlType.torque:
      # disable radar ECU by setting to programming mode
      # if self.radar_disabled == 0 and self.frame > 200:
      #   can_sends.append(create_disable_radar(1))
      #   carlog.error("PSA_DEBUG create_disable_radar")
      #   self.radar_disabled = 1

      # # keep radar ECU disabled by sending tester present
      # # [CLAUDE artiv-nopad] - START
      # # Riga sostituita (paddava a DLC 8, formato diverso da create_disable_radar):
      # #   can_sends.append(make_tester_present_msg(0x6b6, 1, suppress_response=False))
      # if self.frame % 100 == 0 and self.frame>201: # TODO check if disable_radar is sent 100 frames before
      #   can_sends.append(create_tester_present(1))
      #   carlog.error("PSA_DEBUG create_tester_present")
      #   if not self.first_message and self.frame > 1000:
      #     carlog.error("PSA_DEBUG create_tester_present first_message")
      #     can_sends.append(create_request_takeover(self.packer, CS.HS2_DYN_MDD_ETAT_2F6, self.params.EPS_TAKEOVER_TYPE))
      #     self.first_message = True
      # # [CLAUDE artiv-nopad] - END

      if self.frame % self.params.STEER_STEP == 0:
        # [CLAUDE eps-rearm-blackout] - START
        # eps_active "effettivo". Subito dopo l'impulso CS.eps_active e' ancora il
        # valore vecchio (IS_DAT_DIRA a 10 Hz, impulso da 100 ms): senza questo il
        # ramo "EPS attivo" gira lo stesso e manda un 4 ACTIVE fra l'1 dell'impulso e
        # il 2 della scaletta. Appena il calo dell'EPS arriva davvero il blackout si
        # chiude in anticipo, cosi' nel caso normale non costa nulla.
        if self.eps_rearm_confirm > 0:
          if not CS.eps_active:
            self.eps_rearm_confirm = 0   # calo confermato, si torna a fidarsi del CAN
          else:
            self.eps_rearm_confirm -= 1
        eps_active = CS.eps_active and self.eps_rearm_confirm == 0
        # [CLAUDE eps-rearm-blackout] - END
        if not CC.latActive:
          self._reset_lat_state()
        else:
          # [CLAUDE eps-rearm-blackout] - START
          # Riga sostituita (usava CS.eps_active grezzo):
          #   if not CS.eps_active: # and not CS.out.steeringPressed:
          # [CLAUDE eps-rearm-blackout] - END
          if not eps_active: # and not CS.out.steeringPressed:
            # [eps-rearm-ladder] - START
            # self._activate_eps( CS.eps_active)
            # Durante l'impulso l'uscita da ACTIVE e' voluta: la scaletta resta ferma
            # e riparte dal gradino 2 solo a impulso finito. Prima partiva al 2o-3o
            # frame dell'impulso, quindi a fine impulso era a una fase casuale.
            if self.eps_rearm_pulse == 0:
              self._activate_eps()
            # [eps-rearm-ladder] - END

          else:
            ##########
            ### START EPS ACTIVE
            ######
            # EPS is active, proceed with lateral control
            self.send_takeover_req = False
            self.takeover_req_sent = False
            self.lat_activation_frame = 0
            self.status = 4 # 4: EPS ACTIVE
            # [eps-rearm-ladder] - START
            # EPS tornato ACTIVE: la scaletta ha finito il suo lavoro, si azzera
            # cosi' la prossima riattivazione riparte pulita dal gradino 2.
            self._reset_rearm_ladder()
            # [eps-rearm-ladder] - END

            # [eps-rearm] - START
            # Ogni EPS_REARM_PERIOD di assist ininterrotto in rettilineo, arma un
            # impulso breve fuori da ACTIVE (EPS_REARM_STATUS) per far ri-armare l'EPS.
            # Il contatore avanza solo qui (EPS attivo), quindi i 5 s ripartono dalla
            # riattivazione. Se allo scadere non siamo in rettilineo resta carico e
            # l'impulso parte al primo rettilineo utile.
            if self.eps_rearm_pulse == 0:
              self.eps_rearm_counter += 1
              if self.eps_rearm_counter >= self.eps_rearm_period and self._going_straight(CC, CS):
                self.eps_rearm_counter = 0
                self.eps_rearm_pulse = self.eps_rearm_len
                # [CLAUDE eps-rearm-blackout] - START
                # Parte da qui: l'impulso e' piu' corto della latenza con cui l'EPS ci
                # riporta il calo, quindi per i prossimi invii eps_active non e' fidato.
                self.eps_rearm_confirm = self.eps_rearm_blackout
                # [CLAUDE eps-rearm-blackout] - END
                carlog.error(f"PSA_DEBUG eps_rearm curv={CC.actuators.curvature:.5f} angle={CS.out.steeringAngleDeg:.1f} v={CS.out.vEgo:.1f}")
            # [eps-rearm] - END

            if (CS.out.steeringPressed):
              #### DRIVER STEERING DETECTED
              self.apply_torque_factor = self.params.MIN_TORQUE_FACTOR
              temp_driverSteeringTorque = CS.out.steeringTorque
              apply_new_torque_scaled = apply_driver_steer_torque_limits(0, self.apply_torque_last,
                                                              temp_driverSteeringTorque, self.params, self.params.STEER_MAX)
              # [driver-override] - END
            else:
              # --- Requested torque (raw, still float) ------------------------------
              # actuators.torque is the model output in -1..1; scale to counts (x STEER_MAX).
              # Kept as a float here: it feeds the torque-factor curve below.
              #   ex: 0.25 * 250 = 62.5
              actuatorsRequestedTorque = CC.actuators.torque * self.params.STEER_MAX

              # --- Torque factor (dynamic EPS gain, MIN..MAX) -----------------------
              # The EPS multiplies our command by factor/100. Small requests get a low
              # factor (gentle), big ones a high factor, via a slightly convex curve.
              # ratio = normalized request magnitude, clamped 0..1, raised to 1.2.
              #   ex: (62.5/250) ** 1.2 = 0.25 ** 1.2 = 0.19
              ratio = min(1.0, (abs(actuatorsRequestedTorque) / float(self.params.STEER_MAX)) * 1.0) **1.2

              # Lerp ratio onto [MIN_TORQUE_FACTOR, MAX_TORQUE_FACTOR], then clamp.
              #   ex: 15 + 0.19 * (100 - 15) = 31
              self.apply_torque_factor = int(self.params.MIN_TORQUE_FACTOR + ratio * (self.params.MAX_TORQUE_FACTOR - self.params.MIN_TORQUE_FACTOR))
              self.apply_torque_factor = max(self.params.MIN_TORQUE_FACTOR, min(self.apply_torque_factor, self.params.MAX_TORQUE_FACTOR))

              # --- Effective (scaled) torque ---------------------------------------
              # What the wheel actually gets = request * factor/100. Same "force at the
              # wheel" domain as the driver torque, so this is what the limiter compares.
              #   ex: round(62.5 * 31/100) = 19
              new_torque_scaled = int(round(actuatorsRequestedTorque * self.apply_torque_factor / 100))

              # --- Driver-aware rate/override limiter -------------------------------
              # Feed the EFFECTIVE (scaled) command: it is already in the driver-torque
              # domain, so the driver signal needs no conversion. The limiter rate-limits
              # the ramp (STEER_DELTA_UP/DOWN) and backs off when the driver pushes.
              # It always returns an int (see lateral.py).
              temp_driverSteeringTorque = CS.out.steeringTorque
              apply_new_torque_scaled = apply_driver_steer_torque_limits(new_torque_scaled, self.apply_torque_last,
                                                              temp_driverSteeringTorque, self.params, self.params.STEER_MAX)

        # --- Back to a raw CAN command ----------------------------------------
        # The EPS re-applies factor/100, so undo the scaling to recover the raw value
        # to send. can_torque and factor travel as two separate CAN signals. Sent
        # every STEER_STEP frames; psa.h check_relay is set for PSA_LANE_KEEP_ASSIST.
        #   ex: round(19 / 31 * 100) = 61  ->  EPS redoes 61 * 31/100 = 19 (effective)
        if self.apply_torque_factor > 0:
          can_torque = int(round(apply_new_torque_scaled / self.apply_torque_factor *100))
        else:
          can_torque = 0
        # can_sends.append(create_lka_steering(self.packer, CC.latActive, can_torque, self.apply_torque_factor, self.status))
        # [eps-rearm] - START
        send_status = self.status
        if self.eps_rearm_pulse > 0:
          self.eps_rearm_pulse -= 1
          # send_status = 1  # 1: UNSELECTED
          send_status = self.params.EPS_REARM_STATUS  # default 2: SELECTED (enum TX 1010)
        # can_sends.append(create_lka_steering(self.packer, CC.latActive, can_torque, self.apply_torque_factor, self.status,apply_new_torque_scaled))
        can_sends.append(create_lka_steering(self.packer, CC.latActive, can_torque, self.apply_torque_factor, send_status, apply_new_torque_scaled))
        # [eps-rearm] - END
        # [eps-fault] - START
        # Caso estremo: dopo EPS_FAULT_AFTER l'EPS non e' ancora tornato. Avvisa
        # openpilot (interface.py lo travasa in ret.steerFaultTemporary). Il flag si
        # rilascia da solo nel blocco in cima a update(), non qui: appena sale,
        # latActive va a False e questo ramo non gira piu'.
        # [CLAUDE eps-rearm-blackout] - START
        # Riga sostituita (usava CS.eps_active grezzo: durante il blackout il conto
        # eps_rearm_wait avanza gia', quindi qui va usato lo stesso eps_active):
        #   if (self.eps_fault_sends > 0 and CC.latActive and not CS.eps_active
        if (self.eps_fault_sends > 0 and CC.latActive and not eps_active
            and self.eps_rearm_wait >= self.eps_fault_sends and not self.eps_rearm_failed):
          self.eps_rearm_failed = True
          self.eps_fault_hold_cnt = self.eps_fault_hold_frames
          carlog.error(f"PSA_DEBUG eps_fault: riattivazione fallita, EPS fermo da {self.eps_rearm_wait * self.eps_rearm_step:.2f}s")
        # [eps-fault] - END
        # Remember the effective (scaled) value for the next frame's rate limit.
        self.apply_torque_last = apply_new_torque_scaled
        self.apply_can_torque_last = can_torque
        ### END EPS ACTIVE
        ##########

    # [takeover-test] - START
    # Invia una volta la richiesta armata da _activate_eps().
    if (self.params.ENABLE_TAKEOVER_REQUEST and self.send_takeover_req):
      if self.frame % 2 == 0:
        carlog.error("PSA_DEBUG create_request_takeover")
        can_sends.append(create_request_takeover(self.packer, CS.HS2_DYN_MDD_ETAT_2F6, self.params.EPS_TAKEOVER_TYPE))
        self.send_takeover_req = False
    #   self.send_takeover_req = False
    #   self.takeover_req_sent = True
    # [takeover-test] - END

    # if self.car_fingerprint in (CAR.PSA_PEUGEOT_3008,):
    #   if self.frame % 10 == 0:
    #     # send steering wheel hold message
    #     can_sends.append(create_steering_hold(self.packer, CC.latActive, CS.is_dat_dira))

    #  ELKOLED LONGITUDINAL CONTROL

    # TUNING
    # >=-0.5: Engine brakes only
    # <-0.5: Add friction brakes
    # pitch = CC.orientationNED[1] if len(CC.orientationNED) == 3 else 0.0
    # accel_slope = math.sin(pitch) * 9.81
    # accel_cmd = actuators.accel + accel_slope

    # brake_accel = -0.5

    # # torque lookup
    # ACCEL_LOOKUP = [-1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0]
    # TORQUE_LOOKUP = [-400, -300, 120, 350, 550, 800, 1000]

    # # calculate Torque
    # torque_nm = interp(accel_cmd, ACCEL_LOOKUP, TORQUE_LOOKUP)
    # torque = max(-400, min(torque_nm, 1000))

    # braking = accel_cmd < brake_accel and not CS.out.gasPressed
    if self.CP.openpilotLongitudinalControl:
      if CC.hudControl.leadVisible:
        sm.update(0)
        leads_v3 = sm['modelV2'].leadsV3

        if leads_v3 and leads_v3[0].x:
          lead_distance = leads_v3[0].x[0]
          time_gap = lead_distance / max(CS.out.vEgo, 3.0)
          HYSTERESIS = 0.2
          if self.bars > 3:
            self.bars = min(3, int(time_gap))
          elif time_gap > self.bars + 1 + HYSTERESIS:
            self.bars = min(3, self.bars + 1)
          elif time_gap < self.bars - HYSTERESIS:
            self.bars = max(0, self.bars - 1)

      else:
        self.bars = 4

      # disable radar ECU by setting to programming mode
      # if self.radar_disabled == 0:
      #   can_sends.append(create_disable_radar())
      #   self.radar_disabled = 1

    #   # Highest torque seen without gas input: ~1000
    #   # Lowest torque seen without break mode: -560 (but only when transitioning from brake to accel mode, else -248)
    #   # Lowest brake mode accel seen: -4.85m/s²

    #   if self.frame % 2 == 0:
    #     can_sends.append(create_HS2_DYN1_MDD_ETAT_2B6(self.packer, self.frame // 2, actuators.accel, CS.out.cruiseState.enabled, CS.out.gasPressed, braking, CS.out.brakePressed, CS.out.standstill, torque))
    #     can_sends.append(create_HS2_DYN_MDD_ETAT_2F6(self.packer, braking, CC.hudControl.leadVisible, self.bars))

    # # stock long
    # # emulate resume button every 3 seconds to prevent autohold timeout
    # elif CC.latActive and CS.out.standstill and CC.hudControl.leadVisible:
    #   # map: {frame:status} - 0, 1
    #   status = {0: 0, 5: 1}.get(self.frame % 300)
    #   if status is not None:
    #     msg = CS.hs2_dat_mdd_cmd_452
    #     counter = (msg['COUNTER'] + 1) % 16
    #     can_sends.append(create_resume_acc(self.packer, counter, status, msg))

    # #  ELKOLED LONGITUDINAL CONTROL

    if self.car_fingerprint in (CAR.PSA_PEUGEOT_3008,) and self.params.ENABLE_DRIVER_TORQUE:
      if not CC.latActive:
        self.driver_torque_counter = 0
        self.next_driver_torque = random.randint(500, 800)
      else:
        # --- HOLD HANDS (~10 Hz con jitter 8–12 frame) ---
        self.steering_hold_counter += 1
        if self.steering_hold_counter >= self.next_steering_hold:
          can_sends.append(create_steering_hold(self.packer, CC.latActive, CS.is_dat_dira))
          self.steering_hold_counter = 0
          self.next_steering_hold = random.randint(8, 12)
        # --- DRIVER TORQUE (ogni 5–8 s) ---
        self.driver_torque_counter += 1
        if self.driver_torque_counter >= self.next_driver_torque:
          msg = CS.steering
          counter = (msg['COUNTER'] + 1) % 16
          can_sends.append(create_driver_torque(self.packer, CS.steering, counter))
          self.driver_torque_counter = 0
          self.next_driver_torque = random.randint(500, 800)

    # Actuators output
    new_actuators = actuators.as_builder()
    if self.CP.steerControlType == SteerControlType.torque:
      # Keep last applied torque between 20 Hz LKA updates.
      # The EPS maintains assist longer than 50 ms, preventing gaps in actuator output.
      new_actuators.torque = self.apply_torque_last / self.params.STEER_MAX
      new_actuators.torqueOutputCan = self.apply_torque_last
      new_actuators.steeringAngleDeg = float(self.apply_can_torque_last)
      # new_actuators.curvature = temp_driverSteeringTorque   # lo vedi in juggle come carControl.actuatorsOutput.curvature
      # new_actuators.steeringAngleDeg = float(self.apply_torque_factor)

      # if self.frame % 100 == 0:
      #   carlog.error(f"PSA_DEBUG torque={new_actuators.torque:.3f} torque_can={self.apply_torque_last}")

    self.frame += 1
    return new_actuators, can_sends
