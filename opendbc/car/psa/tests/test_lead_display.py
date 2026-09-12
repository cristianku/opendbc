# [lead display] - START
import math
import unittest
from collections import Counter
from types import SimpleNamespace

from opendbc.car.psa.tests.test_longitudinal import LongitudinalHarness


class ModelFeed:
  """In-memory modelV2 input for the real controller and CAN encoder."""
  def __init__(self, distance):
    self.seen = {'modelV2': True}
    self.valid = {'modelV2': True}
    self.alive = {'modelV2': True}
    self.model = SimpleNamespace(leadsV3=[SimpleNamespace(x=[distance])])

  def update(self, timeout):
    pass

  def __getitem__(self, service):
    assert service == 'modelV2'
    return self.model


class TestLeadDisplay(unittest.TestCase):
  def harness(self, distance=30.0, engaged=False):
    h = LongitudinalHarness()
    h.cc.enabled = engaged
    h.cc.longActive = engaged
    h.cc.hudControl.leadVisible = True
    h.controller.model_sm = ModelFeed(distance)
    h.activate()
    h.cs.out.vEgo = 10.0
    return h

  def assert_target(self, h, detected, position):
    _, values = h.emission()
    self.assertEqual(values[0x2F6]['TARGET_DETECTED'], detected)
    self.assertEqual(values[0x2F6]['TARGET_POSITION'], position)
    return values

  def test_distance_buckets_are_encoded_while_cruise_is_disengaged(self):
    # vEgo=10 m/s gives denominator 15; zero distance is a valid closest target.
    for distance, position in ((0, 0), (22.5, 1), (37.5, 2), (60, 3), (300, 3)):
      with self.subTest(distance=distance):
        h = self.harness(distance)
        values = self.assert_target(h, 1, position)
        self.assertEqual(values[0x2B6]['ACC_STATUS'], 2)
        self.assertEqual(values[0x2B6]['POTENTIAL_WHEEL_TORQUE_REQUEST'], 0)
        self.assertEqual(values[0x2B6]['WHEEL_TORQUE_REQUEST'], 0)
        self.assertEqual(values[0x2F6]['MDD_DECEL_CONTROL_REQ'], 0)

  def test_hysteresis_holds_near_thresholds_and_moves_one_position_per_message(self):
    h = self.harness(22.5)
    self.assert_target(h, 1, 1)
    for distance, position in ((32.9, 1), (33.1, 2), (27.1, 2), (26.9, 1), (150, 2), (150, 3), (0, 2), (0, 1), (0, 0)):
      h.controller.model_sm.model.leadsV3[0].x = [distance]
      self.assert_target(h, 1, position)

  def test_same_distance_uses_vehicle_speed_to_select_position(self):
    for speed, position in ((0, 3), (5, 3), (10, 2), (15, 1), (30, 0)):
      with self.subTest(speed=speed):
        h = self.harness(30)
        h.cs.out.vEgo = speed
        self.assert_target(h, 1, position)

  def test_visible_target_does_not_start_substitution_with_alpha_disabled(self):
    h = LongitudinalHarness(experimental=False)
    h.cc.hudControl.leadVisible = True
    h.controller.model_sm = ModelFeed(30)
    for _ in range(120):
      _, messages = h.step()
      self.assertFalse(any(a in (0x2F6, 0x6B6) for a, _, _ in messages))

  def test_loss_of_target_or_invalid_model_clears_display_and_reinitializes_bucket(self):
    cases = ('hidden', 'unseen', 'invalid', 'stale', 'no_subscriber', 'no_leads', 'no_distance',
             'negative_distance', 'nan_distance', 'infinite_distance', 'nan_speed', 'invalid_denominator')
    for case in cases:
      with self.subTest(case=case):
        h = self.harness(60)
        self.assert_target(h, 1, 3)
        feed = h.controller.model_sm
        if case == 'hidden':
          h.cc.hudControl.leadVisible = False
        elif case in ('unseen', 'invalid', 'stale'):
          getattr(feed, {'unseen': 'seen', 'invalid': 'valid', 'stale': 'alive'}[case])['modelV2'] = False
        elif case == 'no_subscriber':
          h.controller.model_sm = None
        elif case == 'no_leads':
          feed.model.leadsV3 = []
        elif case == 'no_distance':
          feed.model.leadsV3[0].x = []
        elif case in ('negative_distance', 'nan_distance', 'infinite_distance'):
          feed.model.leadsV3[0].x = [{'negative_distance': -1, 'nan_distance': math.nan, 'infinite_distance': math.inf}[case]]
        else:
          h.cs.out.vEgo = math.nan if case == 'nan_speed' else -5.0
        self.assert_target(h, 0, 0)  # retain the recorded no-target CAN encoding
        self.assertEqual(h.controller.bars, 4)  # internal Elkoled reset sentinel
        h.controller.model_sm = ModelFeed(7.5)
        h.cc.hudControl.leadVisible = True
        h.cs.out.vEgo = 10.0
        self.assert_target(h, 1, 0)

  def test_target_display_preserves_braking_takeover_and_single_message_cadence(self):
    h = self.harness(30, engaged=True)
    h.cc.actuators.accel = -0.75
    h.controller.takeover_req = 2
    counts, counters = Counter(), []
    for _ in range(100):
      _, messages = h.step()
      self.assertLessEqual(sum(a == 0x2F6 for a, _, _ in messages), 1)
      for address, data, bus in messages:
        counts[address] += 1
        if address == 0x2F6:
          self.assertEqual(bus, 1)
          values = h.decode(address, data)
          self.assertEqual(values['TARGET_DETECTED'], 1)
          self.assertEqual(values['TARGET_POSITION'], 2)
          self.assertEqual(values['MDD_DECEL_CONTROL_REQ'], 1)
          self.assertEqual(values['REQUEST_TAKEOVER'], 2 if len(counters) < 2 else 0)
          self.assertEqual(sum((b >> 4) + (b & 15) for b in data) & 15, 8)
          counters.append(data[6] >> 4)
    self.assertEqual(counts[0x2F6], 50)
    self.assertEqual(counters, [i % 16 for i in range(50)])


if __name__ == '__main__':
  unittest.main()
# [lead display] - END
