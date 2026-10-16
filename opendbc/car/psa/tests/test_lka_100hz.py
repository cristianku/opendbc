# [CLAUDE lka-100hz] - START
import unittest

from opendbc.car import Bus, structs
from opendbc.car.psa.carcontroller import CarController
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.values import CAR


class TestLka100Hz(unittest.TestCase):
  def setUp(self):
    cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_3008)
    cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.PSA_PEUGEOT_3008)
    self.interface = CarInterface(cp, cp_sp)
    self.controller = CarController({Bus.main: 'psa_aee2010_r3'}, cp, cp_sp)
    self.controller.model_sm = None
    self.cc = structs.CarControl()
    self.cc_sp = structs.CarControlSP()
    self.interface.update([(1, [])])

  def step(self, stock_payload=None, speed_kph=None):
    now_nanos = (self.controller.frame + 1) * 10_000_000
    frames = []
    if stock_payload is not None:
      frames.append((0x3F2, bytes.fromhex(stock_payload), 2))
    if speed_kph is not None:
      frames.append(self.controller.packer.make_can_msg('Dyn4_FRE', 0, {
        'P263_VehV_VPsvValWhlFrtL': speed_kph,
        'P264_VehV_VPsvValWhlFrtR': speed_kph,
        'P265_VehV_VPsvValWhlBckL': speed_kph,
        'P266_VehV_VPsvValWhlBckR': speed_kph,
      }))
    self.interface.update([(now_nanos, frames)])
    _, messages = self.controller.update(self.cc.as_reader(), self.cc_sp, self.interface.CS, now_nanos)
    lka = []
    for msg in messages:
      if msg[0] == 0x3F2:
        lka.append(msg)
    return lka

  def test_lka_sent_every_cycle_when_disengaged(self):
    for _ in range(20):
      lka = self.step('0000090008000000')
      self.assertEqual(len(lka), 1)
      self.assertEqual(lka[0][2], 0)

  def test_lka_sent_every_cycle_when_engaged(self):
    self.cc.latActive = True
    for _ in range(20):
      lka = self.step('0000090008000000', speed_kph=60)
      self.assertEqual(len(lka), 1)

  def test_content_changes_only_on_steer_frames(self):
    # During the activation ladder the payload changes at most once every STEER_STEP cycles.
    self.cc.latActive = True
    steer_step = self.controller.params.STEER_STEP
    payloads = []
    for _ in range(4 * steer_step):
      frame = self.controller.frame
      lka = self.step('0000090008000000', speed_kph=60)
      payloads.append((frame, lka[0][1]))
    for i in range(1, len(payloads)):
      frame, payload = payloads[i]
      prev_payload = payloads[i - 1][1]
      if frame % steer_step != 0:
        self.assertEqual(payload, prev_payload, f"payload changed on non-steer frame {frame}")


if __name__ == "__main__":
  unittest.main()
# [CLAUDE lka-100hz] - END


# [CLAUDE lka-100hz-release] - START
from types import SimpleNamespace

from opendbc.can.packer import CANPacker
from opendbc.safety.tests.libsafety import libsafety_py


def lka_fields(data):
  torque = (data[3] << 3) | (data[4] >> 5)
  if torque >= 1024:
    torque = torque - 2048
  factor = (data[5] & 0xFE) >> 1
  status = (data[4] >> 2) & 7
  return torque, factor, status


class ReleaseHarness:
  # Controller with a fake CarState, every LANE_KEEP_ASSIST also checked by the real panda safety.
  def __init__(self, test):
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
    self.cc.actuators.torque = 0.5
    self.packer = CANPacker('psa_aee2010_r3')
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(structs.CarParams.SafetyModel.psa, 0)
    self.safety.init_tests()
    self.safety.set_controls_allowed_lateral(True)

  def step(self):
    # Returns (frame, torque, factor, status) of the LANE_KEEP_ASSIST sent this cycle.
    frame = self.controller.frame
    self.safety.set_timer(frame * 10_000)
    raw = self.cs.steering['DRIVER_TORQUE']
    addr, data, bus = self.packer.make_can_msg('STEERING', 0, {'DRIVER_TORQUE': raw})
    self.test.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, bus, data)))
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
    torque, factor, status = lka_fields(data)
    return frame, torque, factor, status


def apply_condition(h, name):
  if name == 'disengage':
    h.cc.latActive = False
  elif name == 'defect':
    h.cs.eps_state_lka = 4
    h.cs.eps_active = False
  elif name == 'speed':
    h.cs.speed_kph = 40
  elif name == 'eps_left_active':
    h.cs.eps_state_lka = 2
    h.cs.eps_active = False
  elif name == 'driver_override':
    h.cs.steering['DRIVER_TORQUE'] = 18  # raw * 3 = 54 > 50
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
  elif name == 'eps_left_active':
    h.cs.eps_state_lka = 3
    h.cs.eps_active = True
  elif name == 'driver_override':
    h.cs.steering['DRIVER_TORQUE'] = 0
  else:
    raise ValueError(name)


class TestLkaReleaseBetweenSteps(unittest.TestCase):
  CASES = (
    ('disengage', 2),
    ('defect', 2),
    ('speed', 2),
    ('eps_left_active', None),   # None = STATUS unchanged
    ('driver_override', None),
  )

  def test_release_on_cycles_1_to_4_and_recovery_through_safety(self):
    for name, release_status in self.CASES:
      for offset in (1, 2, 3, 4):
        with self.subTest(case=name, offset=offset):
          h = ReleaseHarness(self)
          steer_step = h.controller.params.STEER_STEP

          # Engaged with active torque until a steer frame.
          last = None
          while True:
            last = h.step()
            if last[0] % steer_step == 0 and last[1] != 0 and last[0] >= 100:
              break
          frame, torque, factor, status = last
          self.assertNotEqual(torque, 0)
          self.assertEqual(status, 4)

          # Cycles before the condition: unchanged repeat.
          for _ in range(offset - 1):
            self.assertEqual(h.step()[1:], (torque, factor, status))

          # Condition appears between two steer updates: release right away, until the next update.
          apply_condition(h, name)
          expected_status = status if release_status is None else release_status
          for _ in range(offset, steer_step):
            f, t, fa, st = h.step()
            self.assertNotEqual(f % steer_step, 0)
            self.assertEqual((t, fa, st), (0, 0, expected_status), f"frame {f}")
          self.assertEqual(h.controller.apply_torque_scaled_last, 0)

          # Old torque never comes back while the condition lasts.
          for _ in range(3 * steer_step):
            self.assertEqual(h.step()[1], 0)

          # Recovery: every frame accepted by the real safety, torque active again.
          clear_condition(h, name)
          torques = []
          for _ in range(30 * steer_step):
            torques.append(h.step()[1])
          nonzero = 0
          for t in torques:
            if t != 0:
              nonzero += 1
          self.assertGreater(nonzero, 0, f"{name}: torque never resumed")

  def test_steady_disengaged_is_not_rebuilt(self):
    h = ReleaseHarness(self)
    h.cc.latActive = False
    msgs = []
    for _ in range(20):
      h.step()
      msgs.append(h.controller.lka_msg_last)
    # Between steer frames the very same cached object is repeated.
    steer_step = h.controller.params.STEER_STEP
    for i in range(1, len(msgs)):
      if i % steer_step != 0:
        self.assertIs(msgs[i], msgs[i - 1])

  def test_activation_ladder_not_released_between_steps(self):
    # EPS not active yet: the ladder frames (factor > 0, torque 0) are repeated, not released.
    h = ReleaseHarness(self)
    h.cs.eps_active = False
    h.cs.eps_state_lka = 2
    steer_step = h.controller.params.STEER_STEP
    for _ in range(4 * steer_step):
      f, t, fa, st = h.step()
      if f % steer_step != 0:
        self.assertEqual((t, fa, st), (h.controller.lka_last_torque, h.controller.lka_last_factor,
                                       h.controller.lka_last_status))
    self.assertGreater(h.controller.lka_last_factor, 0)
# [CLAUDE lka-100hz-release] - END
