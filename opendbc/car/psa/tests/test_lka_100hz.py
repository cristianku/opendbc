# [CLAUDE lka-100hz] - START
import unittest
from types import SimpleNamespace

from opendbc.can.packer import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.psa.carcontroller import CarController
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.values import CAR
from opendbc.safety.tests.libsafety import libsafety_py


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

  def test_lka_sent_every_cycle_during_activation(self):
    self.cc.latActive = True
    for _ in range(20):
      lka = self.step('0000090008000000', speed_kph=60)
      self.assertEqual(len(lka), 1)

  def test_content_changes_only_on_steer_frames(self):
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
    steer_step = self.controller.params.STEER_STEP
    while True:
      last = self.step()
      if last[0] % steer_step == 0 and last[1] != 0 and last[0] >= 100:
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
  elif name == 'driver_override':
    h.cs.steering['DRIVER_TORQUE'] = 18  # raw * 3 = 54 > 50
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
  elif name == 'driver_override':
    h.cs.steering['DRIVER_TORQUE'] = 0
  elif name == 'steering_pressed':
    h.cs.out.steeringPressed = False
  elif name == 'brake':
    h.cs.out.brakePressed = False
    h.safety.set_controls_allowed_lateral(True)
  else:
    raise ValueError(name)


class TestLkaReleaseBetweenSteps(unittest.TestCase):
  CASES = (
    ('disengage', 2),
    ('defect', 2),
    ('speed', 2),
    ('speed_51', 2),
    ('eps_left_active', None),   # None = STATUS unchanged
    ('driver_override', None),
    ('steering_pressed', None),
    ('brake', None),
  )

  def test_release_on_cycles_1_to_4_and_recovery_through_safety(self):
    for torque_request in (0.5, -0.5):
      for name, release_status in self.CASES:
        for offset in (1, 2, 3, 4):
          with self.subTest(torque=torque_request, case=name, offset=offset):
            h = ReleaseHarness(self, torque=torque_request)
            steer_step = h.controller.params.STEER_STEP
            frame, torque, factor, status = h.run_until_active_torque()
            self.assertEqual(status, 4)

            for _ in range(offset - 1):
              self.assertEqual(h.step()[1:], (torque, factor, status))

            apply_condition(h, name)
            expected_status = status if release_status is None else release_status
            for _ in range(offset, steer_step):
              f, t, fa, st = h.step()
              self.assertNotEqual(f % steer_step, 0)
              self.assertEqual((t, fa, st), (0, 0, expected_status), f"frame {f}")
            self.assertEqual(h.controller.apply_torque_scaled_last, 0)

            for _ in range(3 * steer_step):
              self.assertEqual(h.step()[1], 0)

            clear_condition(h, name)
            nonzero = 0
            for _ in range(30 * steer_step):
              if h.step()[1] != 0:
                nonzero += 1
            self.assertGreater(nonzero, 0, f"{name}: torque never resumed")

  def test_brake_at_steer_frame_releases_torque(self):
    h = ReleaseHarness(self)
    steer_step = h.controller.params.STEER_STEP
    h.run_until_active_torque()
    for _ in range(steer_step - 1):
      h.step()
    h.cs.out.brakePressed = True
    f, t, fa, st = h.step()
    self.assertEqual(f % steer_step, 0)
    self.assertEqual((t, fa, st), (0, 0, 4))

  def test_short_disengage_during_ladder_restarts_from_3(self):
    h = ReleaseHarness(self)
    h.cs.eps_active = False
    h.cs.eps_state_lka = 2
    first = h.step()
    self.assertEqual(first[3], 3)
    h.cc.latActive = False
    self.assertEqual(h.step()[1:], (0, 0, 2))
    h.cc.latActive = True
    for _ in range(3):
      self.assertEqual(h.step()[1:], (0, 0, 2))
    resumed = h.step()
    self.assertEqual(resumed[0] % h.controller.params.STEER_STEP, 0)
    self.assertEqual(resumed[3], 3)

  def test_short_disengage_with_torque_restarts_cycle(self):
    h = ReleaseHarness(self)
    steer_step = h.controller.params.STEER_STEP
    h.run_until_active_torque()
    h.cc.latActive = False
    self.assertEqual(h.step()[1:], (0, 0, 2))
    h.cc.latActive = True
    for _ in range(steer_step - 2):
      h.step()
    resumed = h.step()
    self.assertEqual(resumed[0] % steer_step, 0)
    self.assertEqual(h.controller.eps_activation_frame, resumed[0])
    self.assertEqual(resumed[3], 4)
    # ramps up again from zero inside the 8/38 envelope (accepted by the safety in step())
    self.assertLessEqual(abs(resumed[1] * resumed[2] / 100), 8.5)

  def test_steady_disengaged_is_not_rebuilt(self):
    h = ReleaseHarness(self)
    h.cc.latActive = False
    msgs = []
    for _ in range(20):
      h.step()
      msgs.append(h.controller.lka_msg_last)
    steer_step = h.controller.params.STEER_STEP
    for i in range(1, len(msgs)):
      if i % steer_step != 0:
        self.assertIs(msgs[i], msgs[i - 1])

  def test_activation_ladder_repeated_not_released(self):
    # EPS not active: ladder frames (factor > 0, torque 0) repeat unchanged; statuses 3, 4, 2, 3.
    h = ReleaseHarness(self)
    h.cs.eps_active = False
    h.cs.eps_state_lka = 2
    steer_step = h.controller.params.STEER_STEP
    statuses = []
    previous = None
    for _ in range(4 * steer_step):
      f, t, fa, st = h.step()
      if f % steer_step == 0:
        statuses.append(st)
      else:
        self.assertEqual(h.last_data, previous)
      previous = h.last_data
    self.assertEqual(statuses, [3, 4, 2, 3])


if __name__ == "__main__":
  unittest.main()
# [CLAUDE lka-100hz] - END
