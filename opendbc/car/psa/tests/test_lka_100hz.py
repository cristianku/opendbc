import unittest
from types import SimpleNamespace

from opendbc.can.packer import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.psa.carcontroller import CarController
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.values import CAR
from opendbc.safety.tests.libsafety import libsafety_py


def lka_fields(data):
  torque = (data[3] << 3) | (data[4] >> 5)
  if torque >= 1024:
    torque = torque - 2048
  factor = (data[5] & 0xFE) >> 1
  status = (data[4] >> 2) & 7
  return torque, factor, status


class ReleaseHarness:
  # Controller with a fake CarState; every LANE_KEEP_ASSIST also goes through the real panda safety.
  def __init__(self, test, torque=0.5):
    self.test = test
    cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_3008)
    cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.PSA_PEUGEOT_3008)
    self.controller = CarController({Bus.main: 'psa_aee2010_r3'}, cp, cp_sp)
    self.controller.model_sm = None
    self.cs = SimpleNamespace(out=structs.CarState(), eps_active=True, eps_state_lka=3,
                              speed_kph=72, is_dat_dira={}, HS2_DYN_MDD_ETAT_2F6={}, steering={'DRIVER_TORQUE': 0})
    self.cs.out.canValid = True
    self.cs.out.vEgo = 20
    self.cs.out.vEgoRaw = 20
    self.cc = structs.CarControl()
    self.cc.latActive = True
    self.cc.actuators.torque = torque
    self.packer = CANPacker('psa_aee2010_r3')
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(structs.CarParams.SafetyModel.psa, 0)
    self.safety.init_tests()
    self.safety.set_controls_allowed_lateral(True)
    self.last_data = None

  def set_driver_torque(self, raw):
    self.cs.steering['DRIVER_TORQUE'] = raw
    self.cs.out.steeringTorque = raw

  def step(self):
    # Returns (frame, torque, factor, status) of the LANE_KEEP_ASSIST sent this cycle.
    frame = self.controller.frame
    self.safety.set_timer(frame * 10_000)
    raw = self.cs.steering['DRIVER_TORQUE']
    addr, data, bus = self.packer.make_can_msg('STEERING', 0, {'DRIVER_TORQUE': raw})
    self.test.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, bus, data)))
    brake = 1 if self.cs.out.brakePressed else 0
    addr, data, bus = self.packer.make_can_msg('Dat_BSI', 2, {'P013_MainBrake': brake})
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, bus, data))
    now = (frame + 1) * 10_000_000
    _, messages = self.controller.update(self.cc.as_reader(), structs.CarControlSP(), self.cs, now)
    lka = []
    for addr, data, bus in messages:
      if addr == 0x3F2:
        lka.append((addr, data, bus))
    self.test.assertEqual(len(lka), 1, f"frame {frame}")
    addr, data, bus = lka[0]
    accepted = self.safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, data))
    self.test.assertTrue(accepted, f"safety rejected frame {frame}: {data.hex()}")
    self.last_data = bytes(data)
    torque, factor, status = lka_fields(data)
    return frame, torque, factor, status

  def run_until_active_torque(self):
    while True:
      last = self.step()
      if last[1] != 0 and last[0] >= 100:
        return last


def apply_condition(h, name):
  if name == 'disengage':
    h.cc.latActive = False
  elif name == 'defect':
    h.cs.eps_state_lka = 4
    h.cs.eps_active = False
  elif name == 'speed':
    h.cs.speed_kph = 40
  elif name == 'speed_51':
    h.cs.speed_kph = 51.0
  elif name == 'eps_left_active':
    h.cs.eps_state_lka = 2
    h.cs.eps_active = False
  elif name == 'steering_pressed':
    h.cs.out.steeringPressed = True
  elif name == 'brake':
    h.cs.out.brakePressed = True
  else:
    raise ValueError(name)


def clear_condition(h, name):
  if name == 'disengage':
    h.cc.latActive = True
  elif name == 'defect':
    h.cs.eps_state_lka = 3
    h.cs.eps_active = True
  elif name == 'speed':
    h.cs.speed_kph = 72
  elif name == 'speed_51':
    h.cs.speed_kph = 72
  elif name == 'eps_left_active':
    h.cs.eps_state_lka = 3
    h.cs.eps_active = True
  elif name == 'steering_pressed':
    h.cs.out.steeringPressed = False
  elif name == 'brake':
    h.cs.out.brakePressed = False
    h.safety.set_controls_allowed_lateral(True)
  else:
    raise ValueError(name)


class TestLka100Hz(unittest.TestCase):
  CASES = (
    ('disengage', 2),
    ('defect', 2),
    ('speed', 2),
    ('speed_51', 2),
    ('eps_left_active', None),   # None: torque 0, ladder keeps its factor and status
    ('steering_pressed', 4),
    ('brake', 4),
  )

  def test_one_full_frame_every_cycle(self):
    h = ReleaseHarness(self)
    for _ in range(300):
      h.step()
    h.cc.latActive = False
    for _ in range(50):
      h.step()

  def test_release_on_next_cycle_at_any_ladder_phase(self):
    ladder_step = ReleaseHarness(self).controller.params.EPS_LADDER_STEP
    for torque_request in (0.5, -0.5):
      for name, release_status in self.CASES:
        for offset in range(ladder_step):
          with self.subTest(torque=torque_request, case=name, offset=offset):
            h = ReleaseHarness(self, torque=torque_request)
            h.run_until_active_torque()
            for _ in range(offset):
              h.step()
            apply_condition(h, name)
            f, t, fa, st = h.step()
            self.assertEqual(t, 0, f"frame {f}")
            if release_status is not None:
              self.assertEqual((fa, st), (0, release_status))
            self.assertEqual(h.controller.apply_torque_scaled_last, 0)
            for _ in range(20):
              self.assertEqual(h.step()[1], 0)

            clear_condition(h, name)
            nonzero = 0
            for _ in range(300):
              if h.step()[1] != 0:
                nonzero += 1
            self.assertGreater(nonzero, 0, f"{name}: torque never resumed")

  def test_torque_moves_every_cycle_within_rate_limits(self):
    h = ReleaseHarness(self, torque=1.0)
    previous = 0
    changes = 0
    for _ in range(300):
      f, t, fa, st = h.step()
      effective = round(t * fa / 100)
      if effective != previous:
        changes += 1
      self.assertLessEqual(effective - previous, h.controller.params.STEER_DELTA_UP + 1)
      previous = effective
    self.assertGreater(changes, 50)

  def test_activation_ladder_at_20hz(self):
    h = ReleaseHarness(self)
    h.cs.eps_active = False
    h.cs.eps_state_lka = 2
    ladder_step = h.controller.params.EPS_LADDER_STEP
    statuses = []
    previous = None
    for _ in range(4 * ladder_step + 1):
      f, t, fa, st = h.step()
      if f % ladder_step == 0:
        statuses.append(st)
      elif previous is not None:
        self.assertEqual(h.last_data, previous)
      previous = h.last_data
    self.assertEqual(statuses[:4], [3, 4, 2, 3])

  def test_short_disengage_restarts_ladder_from_3(self):
    h = ReleaseHarness(self)
    h.cs.eps_active = False
    h.cs.eps_state_lka = 2
    ladder_step = h.controller.params.EPS_LADDER_STEP
    while h.step()[3] != 4:
      pass
    h.cc.latActive = False
    self.assertEqual(h.step()[1:], (0, 0, 2))
    h.cc.latActive = True
    while True:
      f, t, fa, st = h.step()
      if f % ladder_step == 0:
        self.assertEqual(st, 3)
        break
      self.assertEqual(st, 2)


class TestDriverTorqueStandard(unittest.TestCase):
  def test_touch_without_steering_pressed_keeps_torque(self):
    for torque_request in (0.5, -0.5):
      for raw in (18, -18, 25, -25):
        with self.subTest(torque=torque_request, raw=raw):
          h = ReleaseHarness(self, torque=torque_request)
          h.run_until_active_torque()
          h.set_driver_torque(raw)
          for _ in range(4):
            self.assertNotEqual(h.step()[1], 0)

  def test_strong_opposing_driver_torque_is_limited_and_accepted(self):
    for torque_request in (1.0, -1.0):
      with self.subTest(torque=torque_request):
        h = ReleaseHarness(self, torque=torque_request)
        for _ in range(300):
          h.step()
        sign = 1 if torque_request > 0 else -1
        h.set_driver_torque(-sign * 60)
        for _ in range(100):
          f, t, fa, st = h.step()
        # STEER_MAX + (ALLOWANCE - 60) * MULTIPLIER = 150 - 129 = 21
        self.assertLessEqual(abs(t * fa / 100), 21.5)


if __name__ == "__main__":
  unittest.main()
