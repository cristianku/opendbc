from opendbc.can.packer import CANPacker
# [CLAUDE eps-rearm] - START
# from opendbc.car import Bus, structs, make_tester_present_msg
from opendbc.car import Bus, structs, make_tester_present_msg, DT_CTRL
from opendbc.car.common.conversions import Conversions as CV
# [CLAUDE eps-rearm] - END
from opendbc.car.carlog import carlog
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
# from opendbc.car.psa.psacan import create_lka_steering, create_driver_torque, create_steering_hold, create_resume_acc, create_disable_radar, create_HS2_DYN1_MDD_ETAT_2B6, create_HS2_DYN_MDD_ETAT_2F6
# [CLAUDE takeover-test] - START
# from opendbc.car.psa.psacan import create_lka_steering, create_driver_torque, create_steering_hold, create_resume_acc,  create_HS2_DYN1_MDD_ETAT_2B6, create_HS2_DYN_MDD_ETAT_2F6
# [CLAUDE radar-disable] - START
# from opendbc.car.psa.psacan import (create_lka_steering, create_driver_torque, create_steering_hold, create_request_takeover)
from opendbc.car.psa.psacan import (create_lka_steering, create_driver_torque, create_steering_hold, create_request_takeover,
                                    create_disable_radar)
# [CLAUDE radar-disable] - END
# [CLAUDE takeover-test] - END
from opendbc.car.psa.values import CarControllerParams, CAR
# from cereal import messaging
# from numpy import interp

import random
# import math

SteerControlType = structs.CarParams.SteerControlType
# sm = messaging.SubMaster(['modelV2'], poll='modelV2')

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.apply_torque_last = 0
    self.apply_can_torque_last = 0  # raw CAN torque logged to steeringAngleDeg (debug); init so it always exists
    self.apply_torque_factor = 0
    self.apply_torque = 0
    self.status = 2
    self.takeover_req_sent = False
    # this is the frame when the latactive is being pressed
    self.lat_activation_frame  = 0
    self.lat_active_last = False
    self.car_fingerprint = CP.carFingerprint
    self.params = CarControllerParams(CP)
    self.radar_disabled = 0
    self.bars = 4
    self.steering_hold_counter = 0
    self.next_steering_hold = random.randint(8, 12)  # ~10Hz con jitter ±20%
    self.driver_torque_counter = 0
    self.next_driver_torque = random.randint(500, 800)  # 5–8 s @100 Hz
    # [CLAUDE eps-rearm] - START
    # Contatori in "invii LKA" (uno ogni STEER_STEP frame -> 20 Hz).
    self.eps_rearm_step = DT_CTRL * self.params.STEER_STEP                                        # 0.05 s
    self.eps_rearm_period = int(self.params.EPS_REARM_PERIOD / self.eps_rearm_step)                # 100 invii = 5 s
    self.eps_rearm_len = max(1, int(round(self.params.EPS_REARM_LENGTH / self.eps_rearm_step)))    # 2 invii = 100 ms
    self.eps_rearm_counter = 0   # invii LKA con EPS attivo dall'ultimo impulso
    self.eps_rearm_pulse = 0     # invii LKA rimanenti dell'impulso in corso
    # Soglia in km/h in values.py, convertita in m/s una volta sola qui (vEgo e' in m/s).
    self.eps_rearm_min_speed = self.params.EPS_REARM_MIN_SPEED_KPH * CV.KPH_TO_MS    # 80 km/h = 22.2 m/s
    # [CLAUDE eps-rearm] - END
    # [CLAUDE eps-rearm-ladder] - START
    # Scaletta di riattivazione deterministica: ogni gradino tenuto per piu' invii,
    # cosi' l'EPS (che campiona a ~10 Hz) non puo' saltarne uno. Vedi values.py.
    self.eps_rearm_hold = max(1, int(round(self.params.EPS_REARM_STEP_HOLD / self.eps_rearm_step)))              # 3 invii = 150 ms
    self.eps_rearm_ladder_timeout = max(1, int(round(self.params.EPS_REARM_LADDER_TIMEOUT / self.eps_rearm_step)))  # 12 invii = 600 ms
    self.rearm_ladder_pos = None   # None = scaletta ferma; 0/1/2 = gradino 2/3/4
    self.rearm_hold_cnt = 0        # invii gia' fatti sul gradino corrente
    self.rearm_ladder_frames = 0   # invii dall'inizio della scaletta (per il timeout)
    # [CLAUDE eps-rearm-ladder] - END
    # [CLAUDE takeover-test] - START
    # invii dall'inizio della scaletta, NON azzerato dal restart della scaletta:
    # e' il tempo totale in cui l'EPS non e' tornato ACTIVE.
    self.eps_rearm_wait = 0
    self.eps_takeover_sends = int(round(self.params.EPS_TAKEOVER_AFTER / self.eps_rearm_step))  # 4 invii = 200 ms
    # Flag + durata: chi decide alza il flag, l'invio a 50 Hz sta in un blocco a parte
    # e lo consuma da solo (il radar manda lo stesso ID a 50 Hz, vedi values.py).
    self.takeover_req_frames = 0
    self.eps_takeover_hold_frames = int(round(self.params.EPS_TAKEOVER_HOLD / DT_CTRL))  # 100 frame = 1 s
    # test da fermo: richiesta periodica anche senza latActive (0 = disattivato)
    self.takeover_test_frames = int(round(self.params.TAKEOVER_TEST_PERIOD / DT_CTRL))   # 1000 frame = 10 s
    # [CLAUDE takeover-test] - END
    # [CLAUDE radar-disable] - START
    self.radar_disable_frame = int(round(self.params.DISABLE_RADAR_AFTER / DT_CTRL))     # 3000 frame = 30 s
    # [CLAUDE radar-disable] - END
    # [CLAUDE eps-fault] - START
    self.eps_rearm_failed = False   # letto da interface.py -> ret.steerFaultTemporary
    self.eps_fault_sends = int(round(self.params.EPS_FAULT_AFTER / self.eps_rearm_step))     # 50 invii = 2.5 s
    self.eps_fault_hold_frames = int(round(self.params.EPS_FAULT_HOLD / DT_CTRL))            # 150 frame = 1.5 s
    self.eps_fault_hold_cnt = 0
    # [CLAUDE eps-fault] - END

  def _reset_lat_state(self):
    self.status = 2
    self.apply_torque_factor = 0
    self.takeover_req_sent = False
    self.lat_activation_frame = 0
    # [CLAUDE eps-rearm] - START
    self.eps_rearm_counter = 0
    self.eps_rearm_pulse = 0
    # [CLAUDE eps-rearm] - END
    # [CLAUDE eps-rearm-ladder] - START
    self._reset_rearm_ladder()
    # [CLAUDE eps-rearm-ladder] - END

  # [CLAUDE eps-rearm-ladder] - START
  def _reset_rearm_ladder(self):
    self.rearm_ladder_pos = None
    self.rearm_hold_cnt = 0
    self.rearm_ladder_frames = 0
    self.eps_rearm_wait = 0
  # [CLAUDE eps-rearm-ladder] - END

  # [CLAUDE eps-rearm] - START
  def _going_straight(self, CC, CS):
    # Rettilineo + velocita' autostradale + mani a posto: l'unica condizione in cui
    # mollare l'assist per una frazione di secondo e' innocuo.
    return (abs(CC.actuators.curvature) < self.params.EPS_REARM_MAX_CURVATURE and
            abs(CS.out.steeringAngleDeg) < self.params.EPS_REARM_MAX_ANGLE and
            # CS.out.vEgo > self.params.EPS_REARM_MIN_SPEED and   # era in m/s
            CS.out.vEgo > self.eps_rearm_min_speed and
            not CS.out.steeringPressed)
  # [CLAUDE eps-rearm] - END

  def _activate_eps(self, eps_active):
    # Save the frame number when the LKA (steering assist) button is first pressed on the car
    if self.lat_activation_frame == 0:
      # first frame the EPS activate or re activate is sent
      self.lat_activation_frame = self.frame
      # self.takeover_req_sent = False

    if not eps_active: # and not CS.out.steeringPressed:
      #######
      # Alarm - Takeover request!
      # EPS works from 50km/h - Takeover Request if speed is slower than 50
      ######
      # if not self.takeover_req_sent and self.frame % 2 == 0: # 50 Hz
      #   if (self.frame - self.lat_activation_frame) > 10:
        # can_sends.append(create_request_takeover(self.packer, CS.HS2_DYN_MDD_ETAT_2F6,1))
        # self.takeover_req_sent = True

      ######
      # EPS activation sequence 2->3->4 to re-engage
      # STATUS  -  0: UNAVAILABLE, 1: UNSELECTED, 2: READY, 3: AUTHORIZED, 4: ACTIVE
      ######
      # [CLAUDE eps-rearm-ladder] - START
      # Vecchia scaletta: un gradino per ogni invio (50 ms) e ciclo infinito
      # 2->3->4->2. L'EPS campiona a ~10 Hz, quindi ne vedeva uno su due e in
      # ordine sparso -> si riarmava solo per allineamento fortuito di fase
      # (buco misurato 499-2021 ms). Vedi il commento in values.py.
      # self.status = 2 if self.status == 4 else self.status + 1
      # # EPS likes a progressive activation of the Torque Factor
      # self.apply_torque_factor += 10
      # self.apply_torque_factor = min(self.apply_torque_factor, self.params.MAX_TORQUE_FACTOR)
      ladder = self.params.EPS_REARM_LADDER   # (2 SELECTED, 3 AUTHORIZED, 4 ACTIVE)
      if self.rearm_ladder_pos is None:
        # primo invio della scaletta: si riparte sempre dal gradino 2, fase nota
        self.rearm_ladder_pos = 0
        self.rearm_hold_cnt = 0
        self.rearm_ladder_frames = 0
        # Anche il torque factor riparte dal basso, se no la "salita progressiva"
        # non esiste: prima ripartiva dal valore in corso (27-43) e saliva a 100 a
        # vuoto. 10 = primo gradino, come faceva l'aggancio da fermo (0 poi +10).
        self.apply_torque_factor = 10
      else:
        self.rearm_hold_cnt += 1
        self.rearm_ladder_frames += 1
        # [CLAUDE takeover-test] - START
        self.eps_rearm_wait += 1   # non si azzera al restart della scaletta
        # [CLAUDE takeover-test] - END
        if self.rearm_hold_cnt >= self.eps_rearm_hold:
          # gradino tenuto abbastanza a lungo: si sale. Arrivati a 4 si TIENE 4.
          self.rearm_hold_cnt = 0
          if self.rearm_ladder_pos < len(ladder) - 1:
            self.rearm_ladder_pos += 1
            # EPS likes a progressive activation of the Torque Factor: ora un passo
            # per gradino e non uno per invio, cosi' non arriva a 100 a vuoto
            self.apply_torque_factor = min(self.apply_torque_factor + 10, self.params.MAX_TORQUE_FACTOR)
        if self.rearm_ladder_frames >= self.eps_rearm_ladder_timeout:
          # tenere il 4 non e' bastato: si ricomincia la scaletta da 2
          self.rearm_ladder_pos = 0
          self.rearm_hold_cnt = 0
          self.rearm_ladder_frames = 0
      self.status = ladder[self.rearm_ladder_pos]
      # [CLAUDE eps-rearm-ladder] - END

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

    # [CLAUDE eps-fault] - START
    # Il rilascio sta QUI e non nel ramo laterale: appena il flag sale, openpilot
    # porta latActive a False e quel ramo non gira piu' (e _reset_lat_state azzera i
    # contatori), quindi da li' non si spegnerebbe mai.
    if self.eps_fault_hold_cnt > 0:
      self.eps_fault_hold_cnt -= 1
      if self.eps_fault_hold_cnt == 0:
        self.eps_rearm_failed = False   # avviso finito: si ritenta la riattivazione
    # [CLAUDE eps-fault] - END

    # lateral control
    if self.CP.steerControlType == SteerControlType.torque:
      if self.frame % self.params.STEER_STEP == 0:
        if not CC.latActive:
          self._reset_lat_state()
        else:
          if not CS.eps_active: # and not CS.out.steeringPressed:
            # [CLAUDE eps-rearm-ladder] - START
            # self._activate_eps( CS.eps_active)
            # Durante l'impulso l'uscita da ACTIVE e' voluta: la scaletta resta ferma
            # e riparte dal gradino 2 solo a impulso finito. Prima partiva al 2o-3o
            # frame dell'impulso, quindi a fine impulso era a una fase casuale.
            if self.eps_rearm_pulse == 0:
              self._activate_eps(CS.eps_active)
            # [CLAUDE eps-rearm-ladder] - END

          else:
            ##########
            ### START EPS ACTIVE
            ######
            # EPS is active, proceed with lateral control
            self.lat_activation_frame = 0
            self.status = 4 # 4: EPS ACTIVE
            # [CLAUDE eps-rearm-ladder] - START
            # EPS tornato ACTIVE: la scaletta ha finito il suo lavoro, si azzera
            # cosi' la prossima riattivazione riparte pulita dal gradino 2.
            self._reset_rearm_ladder()
            # [CLAUDE eps-rearm-ladder] - END

            # [CLAUDE eps-rearm] - START
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
                carlog.error(f"PSA_DEBUG eps_rearm curv={CC.actuators.curvature:.5f} angle={CS.out.steeringAngleDeg:.1f} v={CS.out.vEgo:.1f}")
            # [CLAUDE eps-rearm] - END

            if (CS.out.steeringPressed):
              #### DRIVER STEERING DETECTED
              # If the driver is applying torque, give up the assist torque to avoid fighting the driver.
              # [CLAUDE driver-override] - START
              # Il gradino secco a zero azzerava anche apply_torque_last (al rilascio la
              # coppia ripartiva da 0 a STEER_DELTA_UP=8 per invio) e mandava
              # TORQUE_FACTOR=0, che per il panda (lka_active = torque_factor != 0) e a
              # quanto pare anche per l'EPS vale "LKA spento".
              # MISURATO su route 00000015 (26 lug 2026, segm. 4-5): steeringPressed e'
              # scattato 3 volte in 120 s (baseline driver torque 36-46 contro soglia 50)
              # e in 1 caso su 3 l'EPS e' uscito da Active proprio al rientro
              # (factor 0 -> 59), costando piu' di un secondo di riattivazione.
              # self.apply_torque_factor = 0
              # apply_new_torque_scaled = 0
              # # apply_new_torque = 0
              # Si cede lo stesso il volante, ma con il target a 0 passato dal rate
              # limiter (scende di STEER_DELTA_DOWN=38 per invio, 1-2 invii) e con il
              # factor tenuto al minimo invece che a zero.
              self.apply_torque_factor = self.params.MIN_TORQUE_FACTOR
              temp_driverSteeringTorque = CS.out.steeringTorque
              apply_new_torque_scaled = apply_driver_steer_torque_limits(0, self.apply_torque_last,
                                                              temp_driverSteeringTorque, self.params, self.params.STEER_MAX)
              # [CLAUDE driver-override] - END
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
        # [CLAUDE eps-rearm] - START
        # STATUS locale: self.status resta 4 e la macchina a stati 2->3->4 di
        # _activate_eps non viene sporcata. Il decremento sta QUI (un invio LKA),
        # cosi' l'impulso finisce sempre anche se l'EPS cade a meta'.
        # TORQUE e TORQUE_FACTOR continuano a viaggiare: l'EPS con STATUS=1 li ignora,
        # ma il rate-limiter del panda (lka_active = torque_factor != 0) non si resetta.
        send_status = self.status
        if self.eps_rearm_pulse > 0:
          self.eps_rearm_pulse -= 1
          # send_status = 1  # 1: UNSELECTED
          send_status = self.params.EPS_REARM_STATUS  # default 2: SELECTED (enum TX 1010)
        # can_sends.append(create_lka_steering(self.packer, CC.latActive, can_torque, self.apply_torque_factor, self.status,apply_new_torque_scaled))
        can_sends.append(create_lka_steering(self.packer, CC.latActive, can_torque, self.apply_torque_factor, send_status, apply_new_torque_scaled))
        # [CLAUDE eps-rearm] - END
        # [CLAUDE takeover-test] - START
        # TEST: se la riattivazione non va a buon fine entro EPS_TAKEOVER_AFTER, alza
        # il flag. L'invio vero (a 50 Hz, come il radar) e' nel blocco piu' sotto.
        if (self.eps_takeover_sends > 0 and CC.latActive and not CS.eps_active
            and self.eps_rearm_wait >= self.eps_takeover_sends):
          if self.takeover_req_frames == 0:
            carlog.error(f"PSA_DEBUG takeover_req: EPS non riarmato da {self.eps_rearm_wait * self.eps_rearm_step:.2f}s")
          self.takeover_req_frames = self.eps_takeover_hold_frames
        # [CLAUDE takeover-test] - END
        # [CLAUDE eps-fault] - START
        # Caso estremo: dopo EPS_FAULT_AFTER l'EPS non e' ancora tornato. Avvisa
        # openpilot (interface.py lo travasa in ret.steerFaultTemporary). Il flag si
        # rilascia da solo nel blocco in cima a update(), non qui: appena sale,
        # latActive va a False e questo ramo non gira piu'.
        if (self.eps_fault_sends > 0 and CC.latActive and not CS.eps_active
            and self.eps_rearm_wait >= self.eps_fault_sends and not self.eps_rearm_failed):
          self.eps_rearm_failed = True
          self.eps_fault_hold_cnt = self.eps_fault_hold_frames
          carlog.error(f"PSA_DEBUG eps_fault: riattivazione fallita, EPS fermo da {self.eps_rearm_wait * self.eps_rearm_step:.2f}s")
        # [CLAUDE eps-fault] - END
        # Remember the effective (scaled) value for the next frame's rate limit.
        self.apply_torque_last = apply_new_torque_scaled
        self.apply_can_torque_last = can_torque
        ### END EPS ACTIVE
        ##########

    # [CLAUDE takeover-test] - START
    # Innesco periodico per il test da fermo: alza il flag a intervalli fissi anche
    # senza latActive, cosi' la richiesta si puo' provare in parcheggio.
    if self.takeover_test_frames > 0 and self.frame > 0 and self.frame % self.takeover_test_frames == 0:
      carlog.error(f"PSA_DEBUG takeover_test: richiesta periodica a t={self.frame * DT_CTRL:.0f}s")
      self.takeover_req_frames = self.eps_takeover_hold_frames

    # Invio della richiesta di takeover: fuori dal blocco LKA (20 Hz) e a 50 Hz come
    # il radar, stessa cadenza dell'emulazione ARTIV di elkoled qui sotto
    # (self.frame % 2). Il flag lo alza chi rileva la mancata riattivazione, questo
    # blocco lo consuma e si spegne da solo.
    if self.takeover_req_frames > 0:
      self.takeover_req_frames -= 1
      if self.frame % 2 == 0:
        can_sends.append(create_request_takeover(self.packer, CS.HS2_DYN_MDD_ETAT_2F6, self.params.EPS_TAKEOVER_TYPE))
    # [CLAUDE takeover-test] - END

    # [CLAUDE radar-disable] - START
    # TEST DA FERMO, dietro flag (default False in values.py): zittisce il radar
    # ARTIV mettendolo in sessione di programmazione e la tiene giu' a 1 Hz.
    # Perde ACC e AEB finche' e' attivo: vedi il commento esteso in values.py.
    if self.params.DISABLE_RADAR_TEST and self.frame >= self.radar_disable_frame:
      if self.radar_disabled == 0:
        can_sends.append(create_disable_radar())
        self.radar_disabled = 1
        carlog.error("PSA_DEBUG radar_disable: ARTIV messo in programming mode")
      elif self.frame % 100 == 0:
        # tester present: senza questo il radar si risveglia da solo dopo ~5 s
        can_sends.append(make_tester_present_msg(0x6b6, 1, suppress_response=False))
    # [CLAUDE radar-disable] - END

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
    # if self.CP.openpilotLongitudinalControl:
    #   if CC.hudControl.leadVisible:
    #     sm.update(0)
    #     leads_v3 = sm['modelV2'].leadsV3
    #     if leads_v3 and leads_v3[0].x:
    #       r = leads_v3[0].x[0] / (5 + CS.out.vEgo)
    #       if self.bars > 3:  # initialize from "no lead"
    #         self.bars = min(3, int(r))
    #       elif r > self.bars + 1.2:
    #         self.bars = min(3, self.bars + 1)
    #       elif r < self.bars - 0.2:
    #         self.bars = max(0, self.bars - 1)
    #   else:
    #     self.bars = 4

    #   # disable radar ECU by setting to programming mode
    #   if self.radar_disabled == 0:
    #     can_sends.append(create_disable_radar())
    #     self.radar_disabled = 1

    #   # keep radar ECU disabled by sending tester present
    #   if self.frame % 100 == 0 and self.frame>0: # TODO check if disable_radar is sent 100 frames before
    #     can_sends.append(make_tester_present_msg(0x6b6, 1, suppress_response=False))

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
    #     # --- HOLD HANDS (~10 Hz con jitter 8–12 frame) ---
        self.steering_hold_counter += 1
        if self.steering_hold_counter >= self.next_steering_hold:
          can_sends.append(create_steering_hold(self.packer, CC.latActive, CS.is_dat_dira))
          self.steering_hold_counter = 0
          self.next_steering_hold = random.randint(8, 12)
    #     # --- DRIVER TORQUE (ogni 5–8 s) ---
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