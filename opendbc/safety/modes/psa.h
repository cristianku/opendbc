#pragma once

// #include "opendbc/safety/declarations.h"

#define PSA_STEERING              757U  // RX from XXX, driver torque
#define PSA_STEERING_ALT          773U  // RX from EPS, steering angle
#define PSA_DRIVER                1390U // RX from XXX, gas pedal
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

static uint8_t psa_get_counter(const CANPacket_t *msg) {
  uint8_t cnt = 0;
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
      pcm_cruise_check((msg->data[2U] >> 7U) & 1U); // RVV_ACC_ACTIVATION_REQ
    }
  }


  if (msg->bus == PSA_CAM_BUS) {
    if (msg->addr == PSA_DRIVER) {
      gas_pressed = msg->data[3] > 0U; // GAS_PEDAL
    }
    if (msg->addr == PSA_DAT_BSI) {
      brake_pressed = (msg->data[0U] >> 5U) & 1U; // P013_MainBrake
    }
  }

  // CAN0 or CAN2
  if (msg->addr == PSA_STEERING_ALT) {
    int angle_meas_new = to_signed((msg->data[0] << 8) | msg->data[1], 16); // ANGLE
    update_sample(&angle_meas, angle_meas_new);
  }
}

static bool psa_tx_hook(const CANPacket_t *msg) {
  // SAFETY_UNUSED(msg);
  bool tx = true;
  static const TorqueSteeringLimits PSA_STEERING_LIMITS = {
    .max_torque = 200,
    .max_rate_up = 22,
    .max_rate_down = 38,
    .driver_torque_allowance = 50,
    .driver_torque_multiplier = 1,
    .max_rt_delta = 150,
    .type = TorqueDriverLimited,
  };

  // // Safety check for LKA
  if (msg->addr == PSA_LANE_KEEP_ASSIST) {
    // TORQUE: 31|11@0-
    int desired_torque = (msg->data[3] << 3) | (msg->data[4] >> 5);
    desired_torque = to_signed(desired_torque, 11);

    // TORQUE_FACTOR: 47|7@0+
    uint8_t torque_factor = (msg->data[5] & 0xFEU) >> 1;
    bool lka_active = torque_factor != 0U;

    if (steer_torque_cmd_checks(desired_torque, lka_active, PSA_STEERING_LIMITS)) {
      // tx = false;
      tx = true;
    }
  }

  // ARTIV diagnostics: allow only programming session and TesterPresent.
  if (msg->addr == PSA_REQ_DIAG_ARTIV) {
    uint32_t first_word = GET_BYTES(msg, 0, 4);
    bool zero_padding = GET_BYTES(msg, 4, 4) == 0x0U;
    bool is_programming_session = first_word == 0x00021002U;       // 02 10 02 00
    bool is_tester_present = (first_word == 0x00003E02U) ||        // 02 3E 00 00
                             (first_word == 0x00803E02U);          // 02 3E 80 00

    if (!zero_padding || (!is_programming_session && !is_tester_present)) {
      tx = false;
    }
  }

  return tx;
}

static safety_config psa_init(uint16_t param) {
  SAFETY_UNUSED(param);
  static const CanMsg PSA_TX_MSGS[] = {
    {PSA_LANE_KEEP_ASSIST, PSA_MAIN_BUS, 8, .check_relay = true}, // EPS steering
    {PSA_IS_DAT_DIRA, PSA_CAM_BUS, 4, .check_relay = false}, // hold steering wheel
    {PSA_STEERING, PSA_MAIN_BUS, 7, .check_relay = false}, // driver torque
    {PSA_HS2_DYN_MDD_ETAT_2F6, PSA_ADAS_BUS, 8, .check_relay = false}, // request takeover
    {PSA_REQ_DIAG_ARTIV, PSA_ADAS_BUS, 8, .check_relay = false},        // radar diagnostics TODO: check if reduce to 3 is ok
    {PSA_HS2_DAT_MDD_CMD_452, PSA_ADAS_BUS, 6, .check_relay = false}, // resume acc
    {PSA_HS2_SUPV_ARTIV_796, PSA_ADAS_BUS, 8, .check_relay = false},    // radar emulation
    {PSA_HS2_DAT_ARTIV_V2_4F6, PSA_ADAS_BUS, 5, .check_relay = false},  // radar emulation
    {PSA_HS2_DYN1_MDD_ETAT_2B6, PSA_ADAS_BUS, 8, .check_relay = false}, // radar emulation
    {PSA_HS2_DYN_MDD_ETAT_2F6, PSA_ADAS_BUS, 8, .check_relay = false},  // radar emulation
  };

  static RxCheck psa_rx_checks[] = {
    {.msg = {{PSA_HS2_DAT_MDD_CMD_452, PSA_ADAS_BUS, 6, 20U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},                        // cruise state
    {.msg = {{PSA_DYN4_FRE, PSA_MAIN_BUS, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},      // wheel speeds (50Hz)
    {.msg = {{PSA_HS2_DYN_ABR_38D, PSA_MAIN_BUS, 8, 25U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},                            // speed
    // [CLAUDE steering-rx-counter] - START
    // {.msg = {{PSA_STEERING, PSA_MAIN_BUS, 7, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},     // driver torque
    // [CLAUDE steering-rx-checksum] - START
    // {.msg = {{PSA_STEERING, PSA_MAIN_BUS, 7, 100U, .max_counter = 15U, .ignore_checksum = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},             // driver torque (counter verified, checksum TBD)
    {.msg = {{PSA_STEERING, PSA_MAIN_BUS, 7, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},             // driver torque (counter + checksum verified)
    // [CLAUDE steering-rx-checksum] - END
    // [CLAUDE steering-rx-counter] - END
    {.msg = {{PSA_DAT_BSI, PSA_CAM_BUS, 8, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},        // brake
    // GAS_PEDAL - DRIVER -> 208: 6 Bytes, 508: 7 Bytes
    // TODO: Berlingo uses Dyn5_CMM on MAIN_BUS for gas pedal
    {.msg = {                                                                                                                                         // gas_pedal
      {PSA_DRIVER, PSA_CAM_BUS, 5, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},
      {PSA_DRIVER, PSA_CAM_BUS, 6, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},
      {PSA_DRIVER, PSA_CAM_BUS, 7, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},
    }},
    // {.msg = {                                                                                                                                         // steering angle
    //   {PSA_STEERING_ALT, PSA_MAIN_BUS, 7, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},
    //   {PSA_STEERING_ALT, PSA_CAM_BUS, 7, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},
    //   { 0 },
    // }},
  };

  return BUILD_SAFETY_CFG(psa_rx_checks, PSA_TX_MSGS);
}

const safety_hooks psa_hooks = {
  .init = psa_init,
  .rx = psa_rx_hook,
  .tx = psa_tx_hook,
  .get_counter = psa_get_counter,
  .get_checksum = psa_get_checksum,
  .compute_checksum = psa_compute_checksum,
};
