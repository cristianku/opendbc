# [eps fault] - START
import pytest

from opendbc.car import Bus, structs
from opendbc.car.psa.carcontroller import CarController
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.values import CAR


@pytest.fixture(params=[CAR.PSA_PEUGEOT_3008, CAR.PSA_CITROEN_C4_SPACETOURER])
def lka(request):
  cp = CarInterface.get_non_essential_params(request.param)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, request.param)
  interface = CarInterface(cp, cp_sp)
  controller = CarController({Bus.main: 'psa_aee2010_r3'}, cp, cp_sp)
  controller.model_sm = None
  controller.frame = 100
  cc = structs.CarControl()
  cc.latActive = True
  cc.actuators.torque = 0.5
  cc.actuators.curvature = 0.005
  interface.update([(1, [])])
  return interface, controller, cc


def update_lka(lka, eps_state, *, stale_fault_flag=False, stale_eps_active=False):
  interface, controller, cc = lka
  controller.frame += (-controller.frame) % controller.params.STEER_STEP
  now_nanos = (controller.frame + 1) * 10_000_000
  frames = [
    controller.packer.make_can_msg('IS_DAT_DIRA', 0, {'EPS_STATE_LKA': eps_state}),
    controller.packer.make_can_msg('Dyn4_FRE', 0, {
      'P263_VehV_VPsvValWhlFrtL': 72,
      'P264_VehV_VPsvValWhlFrtR': 72,
      'P265_VehV_VPsvValWhlBckL': 72,
      'P266_VehV_VPsvValWhlBckR': 72,
    }),
    controller.packer.make_can_msg('LANE_KEEP_ASSIST', 2, {'unknown2': 9, 'STATUS': 2}),
  ]
  interface.update([(now_nanos, frames)])
  if stale_fault_flag:
    interface.CS.out.steerFaultTemporary = False
  if stale_eps_active:
    interface.CS.eps_active = True
  return controller.update(cc.as_reader(), structs.CarControlSP(), interface.CS, now_nanos)


def assert_neutral(messages):
  lka_msg = next(m for m in messages if m[0] == 0x3F2)
  assert lka_msg[2] == 0
  assert lka_msg[1] == bytes.fromhex('0000090008000000')
  assert not any(m[0] == 0x495 and m[2] == 2 for m in messages)


def test_eps_defect_is_temporary_and_clears_with_raw_state(lka):
  interface, _, _ = lka
  for recovered_state in (0, 1, 2, 3):
    update_lka(lka, 4)
    assert interface.CS.eps_state_lka == 4
    assert interface.CS.out.steerFaultTemporary
    assert not interface.CS.out.steerFaultPermanent

    update_lka(lka, recovered_state)
    assert not interface.CS.out.steerFaultTemporary
    assert not interface.CS.out.steerFaultPermanent


@pytest.mark.parametrize('stale_eps_active', [False, True])
def test_raw_defect_blocks_stale_lateral_request_and_hold(lka, stale_eps_active):
  _, controller, cc = lka
  controller.status = 4
  controller.apply_torque_factor = 80
  controller.apply_torque_scaled_last = 100
  controller.eps_activation_frame = 5
  controller.deactivation_in_progress = True
  controller.latActiveLast = True
  controller.steering_hold_counter = controller.next_steering_hold

  actuators, messages = update_lka(lka, 4, stale_fault_flag=True, stale_eps_active=stale_eps_active)

  assert cc.latActive  # The controller must independently reject the stale request.
  assert_neutral(messages)
  assert actuators.torque == 0
  assert actuators.torqueOutputCan == 0
  assert controller.apply_torque_factor == 0
  assert controller.eps_activation_frame == 0
  assert not controller.deactivation_in_progress
  assert not controller.latActiveLast


def test_persistent_defect_never_restarts_activation(lka):
  _, controller, _ = lka
  for _ in range(20):
    actuators, messages = update_lka(lka, 4)
    assert_neutral(messages)
    assert actuators.torque == 0
    assert controller.status == 2
    assert controller.apply_torque_factor == 0
    assert controller.eps_activation_frame == 0


def test_recovery_starts_fresh_cycle_after_fault(lka):
  interface, controller, _ = lka
  update_lka(lka, 3)
  assert controller.eps_activation_frame > 0
  update_lka(lka, 4)
  assert controller.eps_activation_frame == 0

  _, messages = update_lka(lka, 2)
  assert not interface.CS.out.steerFaultTemporary
  activation = next(m for m in messages if m[0] == 0x3F2)
  assert activation[1] == bytes.fromhex('000018000c140000')

  expected_start = controller.frame + (-controller.frame) % controller.params.STEER_STEP
  actuators, _ = update_lka(lka, 3)
  assert controller.eps_activation_frame == expected_start
  assert controller.status == 4
  assert actuators.torque > 0
# [eps fault] - END
