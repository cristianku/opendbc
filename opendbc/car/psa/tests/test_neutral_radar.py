import unittest
import inspect
from collections import Counter
from types import SimpleNamespace

from opendbc.can.packer import CANPacker
from opendbc.can.parser import CANParser
from opendbc.car import Bus, structs
from opendbc.car.psa import psacan
from opendbc.car.psa.carcontroller import CarController
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.values import CAR


RADAR_IDS = {0x2B6, 0x2F6, 0x4F6, 0x796}


class TestRadarMessageParameters(unittest.TestCase):
  def test_each_builder_encodes_all_explicit_inputs_and_requires_them(self):
    cases = [
      ('create_HS2_DYN1_MDD_ETAT_2B6', 0x2B6, {
        'mdd_desired_deceleration': -1.25, 'potential_wheel_torque_request': 2, 'min_time_for_desired_gear': 3.4,
        'gmp_potential_wheel_torque': 568, 'acc_status': 5, 'gmp_wheel_torque': 931, 'wheel_torque_request': 2,
        'auto_braking_status': 6, 'mdd_decel_type': 3, 'mdd_decel_control_req': 1, 'gear_type': 1, 'prefill_request': 1, 'counter': 7,
      }),
      ('create_HS2_DYN_MDD_ETAT_2F6', 0x2F6, {
        'target_detected': 1, 'request_takeover': 2, 'blind_sensor': 1, 'req_visual_coll_alert_arc': 1,
        'req_audio_coll_alert_arc': 1, 'req_haptic_coll_alert_arc': 1, 'inter_vehicle_distance': 42.75, 'arc_status': 12,
        'auto_braking_in_progress': 1, 'aeb_enabled': 1, 'drive_away_request': 1, 'display_intervehicle_time': 2.4,
        'mdd_decel_control_req': 1, 'auto_braking_status': 5, 'counter': 9, 'target_position': 4,
      }),
      ('create_HS2_DAT_ARTIV_V2_4F6', 0x4F6, {
        'time_gap': 1.7, 'distance_gap': 37, 'relative_speed': -5.2, 'artiv_sensor_state': 3, 'target_detected': 1,
        'artiv_target_change_info': 1, 'traffic_direction': 2,
      }),
      ('create_HS2_SUPV_ARTIV_796', 0x796, {
        'fault_code': 0x24, 'status_no_config': 0x12345678, 'status_partial_wakeup_gmp': 0x4567, 'uce_electr_state': 9,
      }),
    ]
    for name, address, values in cases:
      with self.subTest(message=name):
        self.assertTrue(hasattr(psacan, name))
        builder = getattr(psacan, name)
        self.assertIn('bus', inspect.signature(builder).parameters)
        packer = CANPacker('psa_aee2010_r3')
        # Change each field independently to catch swapped or ignored arguments.
        for changed in (None, *values):
          inputs = dict(values)
          if changed is not None:
            inputs[changed] = 0
          message = builder(packer, bus=2, **inputs)
          self.assertEqual((message[0], message[2]), (address, 2))
          parser = CANParser('psa_aee2010_r3', [(address, 1)], 2)
          parser.update([(1_000_000_000, [message])])
          for signal, expected in inputs.items():
            self.assertAlmostEqual(parser.vl[address][signal.upper()], expected, msg=f'{name}: {signal}')
        for missing in values:
          partial = dict(values)
          del partial[missing]
          with self.assertRaises(TypeError, msg=f'{name} must require {missing}'):
            builder(packer, bus=2, **partial)


class TestNeutralRadar(unittest.TestCase):
  def setUp(self):
    cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_3008)
    cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.PSA_PEUGEOT_3008)
    self.controller = CarController({Bus.main: 'psa_aee2010_r3'}, cp, cp_sp)
    self.cs = SimpleNamespace(eps_active=False, out=structs.CarState())
    self.cs.out.standstill = True
    self.cs.out.canValid = True
    self.controller.neutral_radar.process_can([(10_000_000_000, [(0x2B6, bytes.fromhex('fe0000020000030a'), 1)])])
    self.controller.frame = 1000
    self.controller.update(structs.CarControl().as_reader(), structs.CarControlSP(), self.cs, 10_000_000_000)
    self.controller.neutral_radar.process_can([(10_067_000_000, [(0x696, bytes.fromhex('06500200c80014'), 1)])])
    self.previous = []

  def messages_at(self, frame):
    now_nanos = (1011 + frame) * 10_000_000
    received = [(0x212, bytes(8), 1)] + [(a, d, 129) for a, d, _ in self.previous if a in RADAR_IDS]
    if any(a == 0x6B6 for a, _, _ in self.previous):
      received.append((0x696, b'\x02\x7e\x00', 1))
    self.controller.neutral_radar.process_can([(now_nanos, received)])
    self.controller.frame = 1011 + frame
    _, self.previous = self.controller.update(structs.CarControl().as_reader(), structs.CarControlSP(), self.cs, now_nanos)
    return [message for message in self.previous if message[0] in RADAR_IDS]

  def test_fixed_payloads_match_recorded_stationary_inhibited_state(self):
    self.assertEqual(self.messages_at(0), [
      (0x2B6, bytes.fromhex('fe 00 00 02 00 00 03 0a'), 1),
      (0x2F6, bytes.fromhex('00 ff 86 00 f8 60 0f 00'), 1),
      (0x4F6, bytes.fromhex('ff fe 5f fe 00'), 1),
      (0x796, bytes(8), 1),
    ])

  def test_rates_counter_wrap_checksums_and_disabled_requests(self):
    parser = CANParser('psa_aee2010_r3', [(0x2B6, 50), (0x2F6, 50)], 1)
    counts = Counter()
    counters = {0x2B6: [], 0x2F6: []}
    for frame in range(200):
      messages = self.messages_at(frame)
      counts.update(msg[0] for msg in messages)
      parser.update([(1_000_000_000 + frame * 10_000_000, messages)])
      for addr, data, bus in messages:
        self.assertEqual(bus, 1)
        self.assertIn(addr, RADAR_IDS)
        if addr in counters:
          checksum_byte = 7 if addr == 0x2B6 else 6
          counters[addr].append(data[checksum_byte] >> 4)
          self.assertEqual(sum((b >> 4) + (b & 15) for b in data) & 15, 12 if addr == 0x2B6 else 8)
        if addr == 0x2B6:
          self.assertEqual((data[6] >> 6) & 1, (data[7] >> 4) & 1)
          for signal in ('POTENTIAL_WHEEL_TORQUE_REQUEST', 'WHEEL_TORQUE_REQUEST', 'MDD_DECEL_CONTROL_REQ',
                         'MDD_DECEL_TYPE', 'PREFILL_REQUEST'):
            self.assertEqual(parser.vl[addr][signal], 0)
          self.assertEqual(parser.vl[addr]['ACC_STATUS'], 2)
        elif addr == 0x2F6:
          for signal in ('MDD_DECEL_CONTROL_REQ', 'DRIVE_AWAY_REQUEST', 'AUTO_BRAKING_IN_PROGRESS', 'AEB_ENABLED',
                         'REQUEST_TAKEOVER', 'TARGET_DETECTED'):
            self.assertEqual(parser.vl[addr][signal], 0)
    self.assertEqual(counts, {0x2B6: 100, 0x2F6: 100, 0x4F6: 20, 0x796: 2})
    for sequence in counters.values():
      self.assertEqual(sequence[:18], list(range(16)) + [0, 1])
    self.assertTrue(parser.can_valid)


class TestNeutralRadarSession(unittest.TestCase):
  def setUp(self):
    cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_3008)
    cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.PSA_PEUGEOT_3008)
    self.controller = CarController({Bus.main: 'psa_aee2010_r3'}, cp, cp_sp)
    self.assertTrue(hasattr(self.controller, 'neutral_radar'))
    self.radar = self.controller.neutral_radar
    self.cs = SimpleNamespace(eps_active=False, out=structs.CarState())
    self.cs.out.standstill = True
    self.cs.out.canValid = True

  def receive(self, seconds, messages):
    return self.radar.process_can([(round(seconds * 1e9), messages)])

  def step(self, frame, now_nanos, stationary):
    self.controller.frame = frame
    self.cs.out.standstill = stationary
    _, messages = self.controller.update(structs.CarControl().as_reader(), structs.CarControlSP(), self.cs, now_nanos)
    return [message for message in messages if message[0] in RADAR_IDS or message[0] == 0x6B6]

  def request(self):
    self.receive(10, [(0x2B6, bytes.fromhex('fe0000020000030a'), 1)])
    self.controller.frame = 1000
    _, messages = self.controller.update(structs.CarControl().as_reader(), structs.CarControlSP(), self.cs, 10_000_000_000)
    self.assertEqual([m for m in messages if m[0] == 0x6B6], [(0x6B6, b'\x02\x10\x02', 1)])
    self.assertFalse(any(m[0] in RADAR_IDS for m in messages))

  def activate(self):
    self.request()
    self.receive(10.067, [(0x696, bytes.fromhex('06500200c80014'), 1)])
    messages = self.step(1011, 10_110_000_000, True)
    self.assertTrue(self.radar.active)
    self.assertEqual({m[0] for m in messages}, RADAR_IDS)
    return messages

  def test_request_without_positive_response_never_emulates(self):
    self.request()
    self.assertEqual(self.step(1101, 11_010_000_000, True), [])
    self.receive(11.1, [(0x696, bytes.fromhex('06500200c80014'), 1)])
    self.assertEqual(self.step(1120, 11_200_000_000, True), [])

  def test_stale_negative_and_non_single_frame_responses_never_activate(self):
    self.receive(9.9, [(0x696, bytes.fromhex('06500200c80014'), 1)])
    self.request()
    self.receive(10.067, [(0x696, bytes.fromhex('21500200c80014'), 1)])
    self.assertEqual(self.step(1011, 10_110_000_000, True), [])
    self.receive(10.2, [(0x696, bytes.fromhex('037f1022'), 1)])
    self.receive(10.3, [(0x696, bytes.fromhex('06500200c80014'), 1)])
    self.assertEqual(self.step(1040, 10_400_000_000, True), [])

  def test_positive_response_waits_for_stock_radar_silence(self):
    self.request()
    self.receive(10.067, [(0x696, bytes.fromhex('06500200c80014'), 1)])
    self.receive(10.09, [(0x2B6, bytes.fromhex('fe00000200004315'), 1)])
    self.assertEqual(self.step(1011, 10_110_000_000, True), [])
    self.assertEqual({m[0] for m in self.step(1020, 10_200_000_000, True)}, RADAR_IDS)

  def test_can_invalid_before_activation_never_starts_emulation(self):
    self.request()
    self.receive(10.067, [(0x696, bytes.fromhex('06500200c80014'), 1)])
    self.cs.out.canValid = False
    self.controller.frame = 1011
    _, messages = self.controller.update(structs.CarControl().as_reader(), structs.CarControlSP(), self.cs, 10_110_000_000)
    self.assertFalse(any(m[0] in RADAR_IDS for m in messages))
    self.assertFalse(self.radar.active)
    self.assertEqual(self.radar.stop_reason, 'vehicle CAN invalid before emulation')

  def test_stock_return_stops_before_remapping_echoes_in_same_batch(self):
    messages = self.activate()
    echo = [(a, d, 129) for a, d, _ in messages]
    packets = [(10_200_000_000, echo + [(0x4F6, bytes.fromhex('fffe5ffe00'), 1)])]
    self.assertEqual(self.radar.process_can(packets), packets)
    self.assertFalse(self.radar.active)
    self.assertEqual(self.step(1020, 10_200_000_000, True), [])

  def test_motion_stops_emulation_and_keepalive_without_restarting(self):
    self.activate()
    self.assertEqual(self.step(1012, 10_120_000_000, False), [])
    self.assertEqual(self.step(2012, 20_120_000_000, True), [])

  def test_only_real_radar_tx_echoes_feed_parser_during_emulation(self):
    messages = self.activate()
    packets = [(10_120_000_000, [(a, d, 129) for a, d, _ in messages] + [
      (0x452, bytes(6), 129), (0x2B6, bytes(8), 193),
    ])]
    transformed = self.radar.process_can(packets)
    self.assertEqual([m[2] for m in transformed[0][1]], [1, 1, 1, 1, 129, 193])
    self.assertEqual(packets[0][1][0][2], 129)  # original log data are not mutated
    self.assertEqual(self.radar.process_can([]), [])
    self.step(1013, 10_130_000_000, False)
    self.assertEqual(self.radar.process_can(packets), packets)

  def test_keepalive_is_short_and_payloads_ignore_openpilot_commands(self):
    self.activate()
    previous = []
    all_messages = []
    for frame in range(1012, 1212):
      now = frame / 100
      rx = [(0x212, bytes(8), 1)] + [(a, d, 129) for a, d, _ in previous if a in RADAR_IDS]
      if any(a == 0x6B6 for a, _, _ in previous):
        rx.append((0x696, b'\x02\x7e\x00', 1))
      self.receive(now, rx)
      self.controller.frame = frame
      self.controller.takeover_req = 3
      cc = structs.CarControl()
      cc.enabled = True
      cc.actuators.accel = 2.0 if frame % 2 else -3.0
      cc.hudControl.leadVisible = True
      _, sent = self.controller.update(cc.as_reader(), structs.CarControlSP(), self.cs, round(now * 1e9))
      previous = sent
      all_messages.extend(sent)
    self.assertTrue(self.radar.active)
    self.assertEqual([m for m in all_messages if m[0] == 0x6B6], [(0x6B6, b'\x02\x3e\x00', 1)] * 2)
    self.assertFalse(any(m[0] == 0x452 for m in all_messages))
    self.assertEqual(sum(m[0] == 0x2F6 for m in all_messages), 100)
    for addr, data, _ in all_messages:
      if addr == 0x2F6:
        self.assertEqual(data[:6], bytes.fromhex('00ff8600f860'))
      elif addr == 0x2B6:
        self.assertEqual(data[:6], bytes.fromhex('fe0000020000'))

  def test_missing_tx_echoes_end_trial(self):
    self.activate()
    self.receive(10.5, [(0x212, bytes(8), 1)])
    self.assertEqual(self.step(1050, 10_500_000_000, True), [])
    self.assertFalse(self.radar.active)

  def test_missing_keepalive_reply_stops_even_with_working_can_and_echoes(self):
    previous = self.activate()
    for frame in range(1012, 1212):
      rx = [(0x212, bytes(8), 1)] + [(a, d, 129) for a, d, _ in previous if a in RADAR_IDS]
      self.receive(frame / 100, rx)
      previous = self.step(frame, frame * 10_000_000, True)
    self.assertFalse(self.radar.active)
    self.assertEqual(self.radar.stop_reason, 'TesterPresent response timeout')
    self.assertEqual(previous, [])

  def test_missing_physical_bus_stops_even_with_recent_echoes(self):
    previous = self.activate()
    self.receive(10.32, [(a, d, 129) for a, d, _ in previous])
    self.assertEqual(self.step(1032, 10_320_000_000, True), [])
    self.assertEqual(self.radar.stop_reason, 'ADAS bus RX timeout')

  def test_invalid_vehicle_can_stops_after_initial_echo_grace(self):
    previous = self.activate()
    self.cs.out.canValid = False
    for frame in range(1012, 1050):
      self.receive(frame / 100, [(0x212, bytes(8), 1)] + [(a, d, 129) for a, d, _ in previous if a in RADAR_IDS])
      self.controller.frame = frame
      _, previous = self.controller.update(structs.CarControl().as_reader(), structs.CarControlSP(), self.cs, frame * 10_000_000)
    self.assertFalse(self.radar.active)
    self.assertEqual(self.radar.stop_reason, 'vehicle CAN invalid')


class TestNeutralRadarInterface(unittest.TestCase):
  def test_real_tx_echoes_maintain_can_valid_and_missing_echoes_are_not_hidden(self):
    cp = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_3008)
    cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.PSA_PEUGEOT_3008)
    interface = CarInterface(cp, cp_sp)
    self.assertTrue(hasattr(interface.CC, 'neutral_radar'))
    # Register the signals used by the real CarState, then simulate the other ECUs.
    interface.update([(0, [])])
    packers = {bus: CANPacker(parser.dbc_name) for bus, parser in interface.can_parsers.items()}

    def physical_frames(include_radar=False):
      frames = []
      for bus, parser in interface.can_parsers.items():
        for address in parser.addresses:
          if address == 0x696 or (address in RADAR_IDS and not include_radar):
            continue
          frames.append(packers[bus].make_can_msg(address, parser.bus, {}))
      return frames

    for tick in range(990, 1001):
      state, _ = interface.update([(tick * 10_000_000, physical_frames(True))])
    self.assertTrue(state.canValid)
    interface.CC.frame = 1000
    control = structs.CarControl().as_reader()
    _, sent = interface.apply(control, structs.CarControlSP(), 10_000_000_000)
    self.assertIn((0x6B6, b'\x02\x10\x02', 1), sent)
    for tick in range(1001, 1012):
      frames = physical_frames()
      if tick == 1007:
        frames.append((0x696, bytes.fromhex('06500200c80014'), 1))
      interface.update([(tick * 10_000_000, frames)])
      interface.CC.frame = tick
      _, sent = interface.apply(control, structs.CarControlSP(), tick * 10_000_000)
    self.assertTrue(interface.CC.neutral_radar.active)

    radar_echo_counts = Counter()
    for tick in range(1012, 1130):
      frames = physical_frames()
      frames.extend((a, d, 129) for a, d, _ in sent if a in RADAR_IDS)
      radar_echo_counts.update(a for a, _, _ in frames if a in RADAR_IDS)
      if any(a == 0x6B6 for a, _, _ in sent):
        frames.append((0x696, b'\x02\x7e\x00', 1))
      state, _ = interface.update([(tick * 10_000_000, frames)])
      interface.CC.frame = tick
      _, sent = interface.apply(control, structs.CarControlSP(), tick * 10_000_000)
    self.assertTrue(state.canValid)
    adas_parser = interface.can_parsers[Bus.adas]
    for address in (0x2B6, 0x2F6):
      self.assertGreater(radar_echo_counts[address], 50)
      self.assertEqual(adas_parser.message_states[address].counter_fail, 0)

    for tick in range(1130, 1170):
      state, _ = interface.update([(tick * 10_000_000, physical_frames())])
      interface.CC.frame = tick
      interface.apply(control, structs.CarControlSP(), tick * 10_000_000)
    self.assertFalse(state.canValid)
    self.assertFalse(interface.CC.neutral_radar.active)


if __name__ == '__main__':
  unittest.main()
