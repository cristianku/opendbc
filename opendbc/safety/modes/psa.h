#pragma once

// #include "opendbc/safety/declarations.h"

#define PSA_STEERING              757U  // RX from XXX, driver torque
#define PSA_STEERING_ALT          773U  // RX from EPS, steering angle
#define PSA_DRIVER                1390U // RX from XXX, gas pedal
// [acc hold] - START
#define PSA_DYN5_CMM               552U  // RX from engine, physical accelerator on 3008
// [acc hold] - END
#define PSA_DYN4_FRE              781U  // RX from CDS, wheel speeds
#define PSA_HS2_DYN_UCF_MDD_32D   813U  // RX from UC_FREIN, standstill
#define PSA_HS2_DYN_ABR_38D       909U  // RX from UC_FREIN, speed
#define PSA_NEW_MSG_42D           1069U // RX, ACC related
#define PSA_HS2_DAT_MDD_CMD_452   1106U // RX from BSI, cruise state
#define PSA_DAT_BSI               1042U // RX from BSI, brake
#define PSA_LANE_KEEP_ASSIST      1010U // TX from OP,  EPS
#define PSA_IS_DAT_DIRA           1173U // TX from OP,  hold steering wheel
#define PSA_REQ_DIAG_ARTIV        1718U // TX from OP, radar diagnostics
#define PSA_HS2_SUPV_ARTIV_796    1942U // TX from OP, radar emulation
#define PSA_HS2_DAT_ARTIV_V2_4F6  1270U // TX from OP, radar emulation
#define PSA_HS2_DYN1_MDD_ETAT_2B6 694U  // TX from OP, radar emulation
#define PSA_HS2_DYN_MDD_ETAT_2F6  758U  // TX from OP, radar emulation

// CAN bus
#define PSA_MAIN_BUS 0U
#define PSA_ADAS_BUS 1U
#define PSA_CAM_BUS  2U

// [psa longitudinal] - START
#define PSA_LONG_CONTROL 1U  // safetyParam; matches opendbc/car/psa/values.py
static bool psa_long_control = false;
// [psa longitudinal] - END

// <TEST_ANGLE_START>
#define PSA_TEST_ANGLE 2U
static bool psa_test_angle = false;
static bool psa_angle_seen = false;
static bool psa_eps_seen = false;
static bool psa_angle_requested = false;
static bool psa_angle_driver_pressed = false;
static uint32_t psa_angle_ts = 0U;
static uint32_t psa_eps_ts = 0U;
static unsigned int psa_eps_state = 0U;

// Provisional 100 Hz limits; keep synchronized with PSA_TEST_ANGLE_LIMITS.
static const AngleSteeringLimits PSA_ANGLE_LIMITS = {
  .max_angle = 900,
  .angle_deg_to_can = 10,
  .angle_rate_up_lookup = {{0., 5., 25.}, {0.5, 0.3, 0.04}},
  .angle_rate_down_lookup = {{0., 5., 25.}, {1.0, 0.4, 0.06}},
  .frequency = 100,
};
// <TEST_ANGLE_START_END>

// [acc hold] - START
// Shared checks; the gas source is selected separately for each profile.
#define PSA_COMMON_RX_CHECKS \
  {.msg = {{PSA_HS2_DAT_MDD_CMD_452, PSA_ADAS_BUS, 6, 20U, .max_counter = 15U, .ignore_quality_flag = true}, {0}, {0}}}, \
  {.msg = {{PSA_DYN4_FRE, PSA_MAIN_BUS, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}}, \
  {.msg = {{PSA_HS2_DYN_ABR_38D, PSA_MAIN_BUS, 8, 25U, .max_counter = 15U, .ignore_quality_flag = true}, {0}, {0}}}, \
  {.msg = {{PSA_STEERING, PSA_MAIN_BUS, 7, 100U, .max_counter = 15U, .ignore_quality_flag = true}, {0}, {0}}}, \
  {.msg = {{PSA_DAT_BSI, PSA_CAM_BUS, 8, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
// [acc hold] - END

static uint8_t psa_get_counter(const CANPacket_t *msg) {
  uint8_t cnt = 0;
  // <TEST_ANGLE_START>
  if (msg->addr == PSA_STEERING_ALT) {
    return msg->data[4] & 0xFU;
  }
  // <TEST_ANGLE_START_END>
  if (msg->addr == PSA_HS2_DAT_MDD_CMD_452) {
    cnt = (msg->data[3] >> 4) & 0xFU;
  } else if (msg->addr == PSA_HS2_DYN_ABR_38D) {
    cnt = (msg->data[5] >> 4) & 0xFU;
  } else if (msg->addr == PSA_STEERING) {
    cnt = msg->data[0] & 0xFU;  // DBC: COUNTER 3|4@0+ -> low nibble of byte 0 (no >>4 here)
  } else if (msg->addr == PSA_HS2_DYN1_MDD_ETAT_2B6) {
    cnt = (msg->data[7] >> 4) & 0xFU;  // DBC: COUNTER 63|4@0+ -> high nibble of byte 7
  } else if (msg->addr == PSA_HS2_DYN_MDD_ETAT_2F6) {
    cnt = (msg->data[6] >> 4) & 0xFU;  // DBC: COUNTER 55|4@0+ -> high nibble of byte 6
  } else if (msg->addr == PSA_NEW_MSG_42D) {
    cnt = msg->data[2] & 0xFU;  // DBC: COUNTER 16|4@1+ -> low nibble of byte 2
  } else {
  }
  return cnt;
}

static uint32_t psa_get_checksum(const CANPacket_t *msg) {
  uint8_t chksum = 0;
  // <TEST_ANGLE_START>
  if (msg->addr == PSA_STEERING_ALT) {
    return msg->data[4] >> 4;
  }
  // <TEST_ANGLE_START_END>
  if (msg->addr == PSA_HS2_DAT_MDD_CMD_452) {
    chksum = msg->data[5] & 0xFU;
  } else if (msg->addr == PSA_HS2_DYN_ABR_38D) {
    chksum = msg->data[5] & 0xFU;
  } else if (msg->addr == PSA_STEERING) {
    chksum = (msg->data[0] >> 4) & 0xFU;  // DBC: CHECKSUM 7|4@0+ -> HIGH nibble of byte 0
  } else if (msg->addr == PSA_HS2_DYN1_MDD_ETAT_2B6) {
    chksum = msg->data[7] & 0xFU;  // DBC: CHECKSUM 59|4@0+ -> low nibble of byte 7
  } else if (msg->addr == PSA_HS2_DYN_MDD_ETAT_2F6) {
    chksum = msg->data[6] & 0xFU;  // DBC: CHECKSUM 51|4@0+ -> low nibble of byte 6
  } else if (msg->addr == PSA_NEW_MSG_42D) {
    chksum = (msg->data[2] >> 4) & 0xFU;  // DBC: CHECKSUM 20|4@1+ -> high nibble of byte 2
  } else {
  }
  return chksum;
}

// static uint8_t _psa_compute_checksum(const CANPacket_t *msg, uint8_t chk_ini, int chk_pos) {
static uint8_t _psa_compute_checksum(const CANPacket_t *msg, uint8_t chk_ini, int chk_pos, bool chk_high_nibble) {
  int len = GET_LEN(msg);

  uint8_t sum = 0;
  for (int i = 0; i < len; i++) {
    uint8_t b = msg->data[i];

    if (i == chk_pos) {
      // zero the checksum nibble before summing
      // b &= 0xF0U;
      b &= chk_high_nibble ? 0x0FU : 0xF0U;
    }
    sum += (b >> 4) + (b & 0xFU);
  }
  return (chk_ini - sum) & 0xFU;
}

static uint32_t psa_compute_checksum(const CANPacket_t *msg) {
  uint8_t chk = 0;
  // <TEST_ANGLE_START>
  if (msg->addr == PSA_STEERING_ALT) {
    // return _psa_compute_checksum(msg, 0xB, 4, true);
    // 3008 recorded STEERING_ALT uses XOR over bytes 0..4 only, with
    // the checksum nibble cleared. It does not use the generic PSA sum.
    for (int i = 0; i < 5; i++) {
      uint8_t b = msg->data[i];
      if (i == 4) {
        b &= 0xFU;
      }
      chk ^= (b >> 4) ^ (b & 0xFU);
    }
    return chk;
  }
  // <TEST_ANGLE_START_END>
  if (msg->addr == PSA_HS2_DAT_MDD_CMD_452) {
    chk = _psa_compute_checksum(msg, 0x4, 5, false);   // checksum in LOW nibble of byte 5
  } else if (msg->addr == PSA_HS2_DYN_ABR_38D) {
    chk = _psa_compute_checksum(msg, 0x7, 5, false);   // checksum in LOW nibble of byte 5
  } else if (msg->addr == PSA_STEERING) {
    chk = _psa_compute_checksum(msg, 0xB, 0, true);    // default init 0xB, checksum in HIGH nibble of byte 0
  } else if (msg->addr == PSA_HS2_DYN1_MDD_ETAT_2B6) {
    chk = _psa_compute_checksum(msg, 0xC, 7, false);   // checksum in LOW nibble of byte 7
  } else if (msg->addr == PSA_HS2_DYN_MDD_ETAT_2F6) {
    chk = _psa_compute_checksum(msg, 0x8, 6, false);   // checksum in LOW nibble of byte 6
  } else if (msg->addr == PSA_NEW_MSG_42D) {
    chk = _psa_compute_checksum(msg, 0xC, 2, true);    // checksum in HIGH nibble of byte 2
  } else {
  }
  return chk;
}

static void psa_rx_hook(const CANPacket_t *msg) {
  // if (msg->bus == PSA_MAIN_BUS) {
  //   if (msg->addr == PSA_HS2_DYN_ABR_38D) {
  //     int speed = (msg->data[0] << 8) | msg->data[1];
  //     vehicle_moving = speed > 0;
  //     // UPDATE_VEHICLE_SPEED(speed * 0.01 * KPH_TO_MS); // VITESSE_VEHICULE_ROUES
  //   }
  // }

  if (msg->bus == PSA_MAIN_BUS) {
    // [acc hold] - START
    // <TEST_ANGLE_START>
    // if (psa_long_control && (msg->addr == PSA_DYN5_CMM)) {
    if ((psa_long_control || psa_test_angle) && (msg->addr == PSA_DYN5_CMM)) {
    // <TEST_ANGLE_START_END>
      gas_pressed = msg->data[2] > 0U; // P334_ACCPed_Position, same source as carstate.py
    }
    // [acc hold] - END
    // Wheel speeds from Dyn4_FRE - calculate average like carstate.py parse_wheel_speeds
    if (msg->addr == PSA_DYN4_FRE) {
      int fl = (msg->data[0] << 8) | msg->data[1];  // P263_VehV_VPsvValWhlFrtL
      int fr = (msg->data[2] << 8) | msg->data[3];  // P264_VehV_VPsvValWhlFrtR
      int rl = (msg->data[4] << 8) | msg->data[5];  // P265_VehV_VPsvValWhlBckL
      int rr = (msg->data[6] << 8) | msg->data[7];  // P266_VehV_VPsvValWhlBckR
      // Average of 4 wheel speeds, scale 0.01 km/h, convert to m/s (use float division)
      float speed_ms = (fl + fr + rl + rr) * 0.0025 * KPH_TO_MS;
      vehicle_moving = speed_ms > 0.1 * KPH_TO_MS;
      // [CLAUDE standstill-threshold] - END
      UPDATE_VEHICLE_SPEED(speed_ms);

    }
  }

  if (msg->bus == PSA_ADAS_BUS) {
    if (msg->addr == PSA_HS2_DAT_MDD_CMD_452) {
      // LONGITUDINAL_REGULATION_TYPE: 0 off, 1 RVV, 2 limiter, 3 ACC.
      const unsigned int regulation_type = msg->data[0] & 3U;
      acc_main_on = (regulation_type == 1U) || (regulation_type == 3U);
      pcm_cruise_check((msg->data[2U] >> 7U) & 1U); // RVV_ACC_ACTIVATION_REQ
    }
  }


  if (msg->bus == PSA_CAM_BUS) {
    // [acc hold] - START
    // <TEST_ANGLE_START>
    // if (!psa_long_control && (msg->addr == PSA_DRIVER)) {
    if (!psa_long_control && !psa_test_angle && (msg->addr == PSA_DRIVER)) {
    // <TEST_ANGLE_START_END>
      gas_pressed = msg->data[3] > 0U; // GAS_PEDAL
    }
    // [acc hold] - END
    if (msg->addr == PSA_DAT_BSI) {
      brake_pressed = (msg->data[0U] >> 5U) & 1U; // P013_MainBrake
    }
  }

  // CAN0 or CAN2
  // <TEST_ANGLE_START>
  // if (msg->addr == PSA_STEERING_ALT) {
  if ((msg->addr == PSA_STEERING_ALT) && (!psa_test_angle || (msg->bus == PSA_MAIN_BUS))) {
  // <TEST_ANGLE_START_END>
    int angle_meas_new = to_signed((msg->data[0] << 8) | msg->data[1], 16); // ANGLE
    update_sample(&angle_meas, angle_meas_new);
    // <TEST_ANGLE_START>
    psa_angle_seen = true;
    psa_angle_ts = microsecond_timer_get();
    // <TEST_ANGLE_START_END>
  }
  // <TEST_ANGLE_START>
  // if (psa_test_angle && (msg->bus == PSA_MAIN_BUS)) {
  if (msg->bus == PSA_MAIN_BUS) {
    if (psa_test_angle && (msg->addr == PSA_IS_DAT_DIRA)) {
      psa_eps_state = (msg->data[2] >> 2) & 7U;
      psa_eps_seen = true;
      psa_eps_ts = microsecond_timer_get();
    }
    if (msg->addr == PSA_STEERING) {
      // CarState's 3008 driver torque uses raw * 3 with a threshold of 50.
      int driver_torque = to_signed(msg->data[1], 8) * 3;
      psa_angle_driver_pressed = (driver_torque > 50) || (driver_torque < -50);
      update_sample(&torque_driver, driver_torque);
    }
  }
  // <TEST_ANGLE_START_END>
}

static bool psa_tx_hook(const CANPacket_t *msg) {
  // SAFETY_UNUSED(msg);
  bool tx = true;
  static const TorqueSteeringLimits PSA_STEERING_LIMITS = {
    // <TEST_ANGLE_START>
    // .max_torque = 200,
    // .max_rate_up = 22,
    // Effective torque, matching CarControllerParams after TORQUE_FACTOR.
    .max_torque = 150,
    .max_rate_up = 8,
    // <TEST_ANGLE_START_END>
    .max_rate_down = 38,
    .driver_torque_allowance = 50,
    .driver_torque_multiplier = 1,
    .max_rt_delta = 150,
    .type = TorqueDriverLimited,
  };

  // [psa longitudinal] - START
  const bool longitudinal_allowed = psa_long_control && get_longitudinal_allowed() && !brake_pressed_prev;
  if (msg->addr == PSA_HS2_DYN1_MDD_ETAT_2B6) {
    const unsigned int accel = msg->data[0];                       // raw * 0.05 - 10.65 m/s^2
    const unsigned int potential_req = msg->data[1] & 3U;
    const unsigned int min_time = msg->data[1] >> 2;
    const unsigned int potential = (msg->data[2] << 4) | (msg->data[3] >> 4); // raw * 4 - 4000 Nm
    const unsigned int status = msg->data[3] & 15U;
    const unsigned int wheel = (msg->data[4] << 6) | (msg->data[5] >> 2);     // raw - 4000 Nm
    const unsigned int wheel_req = msg->data[5] & 3U;
    const unsigned int auto_braking_status = msg->data[6] & 7U;
    const unsigned int decel_type = (msg->data[6] >> 3) & 3U;
    const bool decel_req = (msg->data[6] & 0x20U) != 0U;
    const unsigned int gear_type = (msg->data[6] >> 6) & 1U;
    const bool prefill = (msg->data[6] & 0x80U) != 0U;

    const bool no_torque = (potential == 0U) && (wheel == 0U) && (wheel_req == 0U);
    const bool no_decel = (accel == 254U) && (decel_type == 0U) && !decel_req;
    // [acc hold] - START
    // Suspended carries no actuation, including across pedal release/disengagement.
    const bool inactive = no_torque && no_decel && (potential_req == 0U) &&
                          (min_time == 0U) && ((status == 2U) || (status == 3U) || (psa_long_control && (status == 5U)));
    // [acc hold] - END
    // Bounds of the offline Elkoled-derived prototype, not calibrated vehicle limits.
    const bool gmp = (potential_req == 1U) && (wheel_req == 1U) && no_decel &&
                     (potential >= 900U) && (potential <= 1250U) &&
                     (wheel >= 3600U) && (wheel <= 5000U) && (min_time == 62U);
    // [brake limit] - START
    // -2 .. 0 m/s^2, including light braking/speed holding. Provisional service-brake
    // limit, matching LongitudinalParams.BRAKE_MIN_ACCEL; all actuation gates apply.
    const bool braking = (potential_req == 2U) && no_torque && (min_time == 0U) &&
                         (decel_type == 1U) && decel_req && (accel >= 173U) && (accel <= 213U);
    // [brake limit] - END
    tx = !prefill && (auto_braking_status == 3U) && (gear_type == (psa_get_counter(msg) & 1U)) &&
         (psa_get_checksum(msg) == psa_compute_checksum(msg)) &&
         (inactive || (longitudinal_allowed && (status == 4U) && (gmp || braking)));
  }

  if (msg->addr == PSA_HS2_DYN_MDD_ETAT_2F6) {
    const unsigned int takeover = (msg->data[0] >> 1) & 3U;
    const bool aeb_or_auto_braking = (msg->data[2] & 0x30U) != 0U;
    const bool drive_away = (msg->data[4] & 2U) != 0U;
    const bool decel_req = (msg->data[5] & 4U) != 0U;
    tx = (takeover <= 2U) && !aeb_or_auto_braking && !drive_away &&
         (!decel_req || longitudinal_allowed) && (psa_get_checksum(msg) == psa_compute_checksum(msg));
  }
  // [psa longitudinal] - END

  // // Safety check for LKA
  if (msg->addr == PSA_LANE_KEEP_ASSIST) {
    // TORQUE: 31|11@0-
    int desired_torque = (msg->data[3] << 3) | (msg->data[4] >> 5);
    desired_torque = to_signed(desired_torque, 11);

    // TORQUE_FACTOR: 47|7@0+
    uint8_t torque_factor = (msg->data[5] & 0xFEU) >> 1;
    bool lka_active = torque_factor != 0U;

    // <TEST_ANGLE_START>
    // if (steer_torque_cmd_checks(desired_torque, lka_active, PSA_STEERING_LIMITS)) {
    //   // tx = false;
    //   tx = true;
    // }
    if (psa_test_angle) {
      int desired_angle = to_signed((msg->data[6] << 6) | (msg->data[7] >> 2), 14);
      unsigned int status = (msg->data[4] >> 2) & 7U;
      bool shape_valid = (msg->data[0] == 0x40U) && (msg->data[2] == 0U) &&
                         (desired_torque == 0) && ((msg->data[4] & 3U) == 0U) &&
                         ((msg->data[5] & 1U) != 0U) && ((msg->data[7] & 3U) == 0U) &&
                         (torque_factor <= 100U) &&
                         (lka_active ? ((status >= 2U) && (status <= 4U)) : (status == 0U)) &&
                         ((msg->data[1] >> 4) == _psa_compute_checksum(msg, 0xB, 1, true));
      uint32_t now = microsecond_timer_get();
      bool feedback_valid = psa_angle_seen && psa_eps_seen &&
                            (safety_get_ts_elapsed(now, psa_angle_ts) <= 100000U) &&
                            (safety_get_ts_elapsed(now, psa_eps_ts) <= 1000000U);
      // A rising request must start at the measured angle, never at an arbitrary
      // target. This also initializes the rate limiter after an inactive interval.
      bool first_request = lka_active && !psa_angle_requested;
      bool start_violation = first_request && steer_angle_cmd_inactive_check(desired_angle, PSA_ANGLE_LIMITS.max_angle);
      if (first_request) {
        desired_angle_last = SAFETY_CLAMP(angle_meas.values[0], -PSA_ANGLE_LIMITS.max_angle, PSA_ANGLE_LIMITS.max_angle);
      }
      bool violation = steer_angle_cmd_checks(desired_angle, lka_active, PSA_ANGLE_LIMITS);
      violation |= safety_max_limit_check(desired_angle, PSA_ANGLE_LIMITS.max_angle, -PSA_ANGLE_LIMITS.max_angle);
      tx = shape_valid && !start_violation && !violation &&
           (!lka_active || (feedback_valid && (psa_eps_state < 4U) && !psa_angle_driver_pressed && !brake_pressed_prev &&
                           !safety_max_limit_check(angle_meas.values[0], PSA_ANGLE_LIMITS.max_angle, -PSA_ANGLE_LIMITS.max_angle)));
      psa_angle_requested = tx && lka_active;
      if (!tx) {
        desired_angle_last = SAFETY_CLAMP(angle_meas.values[0], -PSA_ANGLE_LIMITS.max_angle, PSA_ANGLE_LIMITS.max_angle);
      }
    } else {
      // if (steer_torque_cmd_checks(desired_torque, lka_active, PSA_STEERING_LIMITS)) {
      //   tx = true;
      // }
      // The legacy controller limits effective torque, then divides by factor.
      // E.g. raw 448 at factor 25 is effective 112, a valid release from 150.
      int product = desired_torque * torque_factor;
      int effective_torque = (SAFETY_ABS(product) + 50) / 100;
      if (product < 0) {
        effective_torque = -effective_torque;
      }
      bool shape_valid = ((msg->data[0] & 0x40U) == 0U) && ((msg->data[5] & 1U) == 0U) &&
                         (msg->data[6] == 0U) && ((msg->data[7] & 0xFCU) == 0U) &&
                         (torque_factor <= 100U) && (lka_active || (desired_torque == 0));
      // Zero torque is an immediate release (driver override, rearm, disengage).
      if (shape_valid && (desired_torque == 0)) {
        desired_torque_last = 0;
        rt_torque_last = 0;
        ts_torque_check_last = microsecond_timer_get();
      }
      // The generic driver check permits faster decreases absent opposition;
      // also enforce the controller's normal 8/38 envelope in effective units.
      int highest = desired_torque_last > 0 ? desired_torque_last + PSA_STEERING_LIMITS.max_rate_up :
                    SAFETY_MIN(desired_torque_last + PSA_STEERING_LIMITS.max_rate_down, PSA_STEERING_LIMITS.max_rate_up);
      int lowest = desired_torque_last > 0 ?
                   SAFETY_MAX(desired_torque_last - PSA_STEERING_LIMITS.max_rate_down, -PSA_STEERING_LIMITS.max_rate_up) :
                   desired_torque_last - PSA_STEERING_LIMITS.max_rate_up;
      bool violation = safety_max_limit_check(effective_torque, highest, lowest);
      violation |= steer_torque_cmd_checks(effective_torque, lka_active, PSA_STEERING_LIMITS);
      tx = shape_valid && !violation &&
           ((desired_torque == 0) || ((controls_allowed || controls_allowed_lateral) &&
                                    !brake_pressed_prev && !psa_angle_driver_pressed));
      if (!tx) {
        desired_torque_last = 0;
        rt_torque_last = 0;
        ts_torque_check_last = microsecond_timer_get();
      }
    }
    // <TEST_ANGLE_START_END>
  }

  // <TEST_ANGLE_START>
  // if (psa_test_angle && ((msg->addr == PSA_IS_DAT_DIRA) || (msg->addr == PSA_STEERING))) {
  if ((psa_test_angle && (msg->addr == PSA_IS_DAT_DIRA)) || (msg->addr == PSA_STEERING)) {
    tx = false;  // Keep physical EPS and driver feedback; no synthetic hands-on.
  }
  // <TEST_ANGLE_START_END>

  // [artiv probe] - START
  // ARTIV diagnostics: allow exact unpadded TesterPresent and programming requests.
  if (msg->addr == PSA_REQ_DIAG_ARTIV) {
    if (GET_LEN(msg) == 3) {
      bool is_tester_present = (msg->data[1] == 0x3EU) && (msg->data[2] == 0x00U);
      bool is_programming_session = (msg->data[1] == 0x10U) && (msg->data[2] == 0x02U);
      if ((msg->data[0] != 0x02U) || (!is_tester_present && !is_programming_session)) {
        tx = false;
      }
    } else if (GET_LEN(msg) == 8) {
      uint32_t first_word = GET_BYTES(msg, 0, 4);
      bool zero_padding = GET_BYTES(msg, 4, 4) == 0x0U;
      bool is_programming_session = first_word == 0x00021002U;       // 02 10 02 00
      bool is_tester_present = (first_word == 0x00003E02U) ||        // 02 3E 00 00
                               (first_word == 0x00803E02U);          // 02 3E 80 00

      if (!zero_padding || (!is_programming_session && !is_tester_present)) {
        tx = false;
      }
    } else {
      tx = false;
    }
  }
  // [artiv probe] - END

  return tx;
}

static safety_config psa_init(uint16_t param) {
  // [psa longitudinal] - START
  psa_long_control = GET_FLAG(param, PSA_LONG_CONTROL);
  // [psa longitudinal] - END
  // <TEST_ANGLE_START>
  psa_test_angle = GET_FLAG(param, PSA_TEST_ANGLE);
  psa_angle_seen = false;
  psa_eps_seen = false;
  psa_angle_requested = false;
  psa_angle_driver_pressed = false;
  psa_angle_ts = 0U;
  psa_eps_ts = 0U;
  psa_eps_state = 0U;
  // <TEST_ANGLE_START_END>
  static const CanMsg PSA_TX_MSGS[] = {
    {PSA_LANE_KEEP_ASSIST, PSA_MAIN_BUS, 8, .check_relay = true}, // EPS steering
    {PSA_IS_DAT_DIRA, PSA_CAM_BUS, 4, .check_relay = false}, // hold steering wheel
    {PSA_STEERING, PSA_MAIN_BUS, 7, .check_relay = false}, // driver torque
    {PSA_HS2_DYN_MDD_ETAT_2F6, PSA_ADAS_BUS, 8, .check_relay = false}, // request takeover
    {PSA_REQ_DIAG_ARTIV, PSA_ADAS_BUS, 8, .check_relay = false},        // radar diagnostics TODO: check if reduce to 3 is ok
    // [artiv probe] - START
    {PSA_REQ_DIAG_ARTIV, PSA_ADAS_BUS, 3, .check_relay = false},        // exact unpadded TesterPresent probe
    // [artiv probe] - END
    {PSA_HS2_DAT_MDD_CMD_452, PSA_ADAS_BUS, 6, .check_relay = false}, // resume acc
    {PSA_HS2_SUPV_ARTIV_796, PSA_ADAS_BUS, 8, .check_relay = false},    // radar emulation
    {PSA_HS2_DAT_ARTIV_V2_4F6, PSA_ADAS_BUS, 5, .check_relay = false},  // radar emulation
    {PSA_HS2_DYN1_MDD_ETAT_2B6, PSA_ADAS_BUS, 8, .check_relay = false}, // radar emulation
    {PSA_HS2_DYN_MDD_ETAT_2F6, PSA_ADAS_BUS, 8, .check_relay = false},  // radar emulation
  };

  // [acc hold] - START
  // Require the physical 100 Hz pedal for the 3008 longitudinal profile; do not
  // allow the constant-zero DRIVER message to satisfy this receive check instead.
  static RxCheck psa_long_rx_checks[] = {
    PSA_COMMON_RX_CHECKS
    {.msg = {{PSA_DYN5_CMM, PSA_MAIN_BUS, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
  };
  static RxCheck psa_rx_checks[] = {
    PSA_COMMON_RX_CHECKS
    {.msg = {
      {PSA_DRIVER, PSA_CAM_BUS, 5, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},
      {PSA_DRIVER, PSA_CAM_BUS, 6, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},
      {PSA_DRIVER, PSA_CAM_BUS, 7, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},
    }},
  };

  // <TEST_ANGLE_START>
  // return psa_long_control ? BUILD_SAFETY_CFG(psa_long_rx_checks, PSA_TX_MSGS) : BUILD_SAFETY_CFG(psa_rx_checks, PSA_TX_MSGS);
  static RxCheck psa_angle_rx_checks[] = {
    PSA_COMMON_RX_CHECKS
    {.msg = {{PSA_DYN5_CMM, PSA_MAIN_BUS, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{PSA_STEERING_ALT, PSA_MAIN_BUS, 7, 100U, .max_counter = 15U, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{PSA_IS_DAT_DIRA, PSA_MAIN_BUS, 4, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
  };
  if (psa_test_angle) {
    return BUILD_SAFETY_CFG(psa_angle_rx_checks, PSA_TX_MSGS);
  }
  return psa_long_control ? BUILD_SAFETY_CFG(psa_long_rx_checks, PSA_TX_MSGS) : BUILD_SAFETY_CFG(psa_rx_checks, PSA_TX_MSGS);
  // <TEST_ANGLE_START_END>
  // [acc hold] - END
}

const safety_hooks psa_hooks = {
  .init = psa_init,
  .rx = psa_rx_hook,
  .tx = psa_tx_hook,
  .get_counter = psa_get_counter,
  .get_checksum = psa_get_checksum,
  .compute_checksum = psa_compute_checksum,
};
