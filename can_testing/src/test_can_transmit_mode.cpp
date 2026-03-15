#include "supervisor.h"
#include "test_can_transmit_mode.h"
#include <Arduino.h>
#include <FlexCAN_T4.h>

#define SEND_TORQUE
#define SEND_TELEMETRY
#define CAN_TX_PROOF_SUMMARY 1

static constexpr float TORQUE_START_LEFT_NM = 2.0f;
static constexpr float TORQUE_START_RIGHT_NM = -2.0f;

static int report_counter = 0;
static bool first_entry = true;
static uint32_t start_time_us = 0;
static uint32_t last_posvel_rx_used_L_us = 0;
static uint32_t last_posvel_rx_used_R_us = 0;

struct CanTxProofStats {
  uint32_t tx_attempts = 0;
  uint32_t tx_ok = 0;
  uint32_t tx_fail = 0;
  uint32_t last_report_us = 0;
};
static CanTxProofStats g_can_tx_proof;

static bool ESC_torque_cmd(Supervisor_typedef *sup,
                           FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can,
                           uint8_t esc_num,
                           float torque) {
  if (!sup) return false;
  if (esc_num >= sup->esc_count) return false;

  CAN_message_t msg;
  msg.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, sup->esc[esc_num].config.node_id);
  msg.len = 8;
  msg.flags.extended = 1;
  canPackFloat(torque, msg.buf);
  canPackFloat(0.0f, msg.buf + 4);
#ifdef SEND_TORQUE
  return can.write(msg);
#else
  (void)can;
  return false;
#endif
}

void test_can_transmit_mode(Supervisor_typedef *sup,
                            FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can) {
  if (!sup) return;

  if (!sup->esc[0].state.alive) {
    ESC_torque_cmd(sup, can, 0, 0.0f);
    ESC_torque_cmd(sup, can, 1, 0.0f);

    Serial.println("balance exit: left ESC not alive -> idle");
    sup->mode = SUP_MODE_IDLE;
    first_entry = true;
    last_posvel_rx_used_L_us = 0;
    last_posvel_rx_used_R_us = 0;
    return;
  }

  if (first_entry) {
    first_entry = false;
    start_time_us = micros();
    last_posvel_rx_used_L_us = 0;
    last_posvel_rx_used_R_us = 0;
    g_can_tx_proof = CanTxProofStats{};
    g_can_tx_proof.last_report_us = start_time_us;
    Serial.println("{\"cmd\":\"PRINT\",\"note\":\"Balance mode started\"}");
  }

  const uint32_t elapsed_us = micros() - start_time_us;

  const float pos_L = sup->esc[0].state.pos_rad;
  const float pos_R = sup->esc[1].state.pos_rad;
  const float vel_L = sup->esc[0].state.vel_rad_s;
  const float vel_R = sup->esc[1].state.vel_rad_s;

  const float dt_loop_s = CONTROL_PERIOD_US * 1e-6f;
  const uint32_t pos_L_us = sup->esc[0].status.last_update_us;
  const uint32_t pos_R_us = sup->esc[1].status.last_update_us;
  const bool new_pos_L = (pos_L_us != 0u) && (pos_L_us != last_posvel_rx_used_L_us);
  const bool new_pos_R = (pos_R_us != 0u) && (pos_R_us != last_posvel_rx_used_R_us);
  const bool new_pos = new_pos_L || new_pos_R;

  float dt_pos_L_s = dt_loop_s;
  float dt_pos_R_s = dt_loop_s;
  if (new_pos_L) {
    dt_pos_L_s = (last_posvel_rx_used_L_us != 0u)
                   ? ((float)((uint32_t)(pos_L_us - last_posvel_rx_used_L_us)) * 1e-6f)
                   : 0.002f;
    last_posvel_rx_used_L_us = pos_L_us;
  }
  if (new_pos_R) {
    dt_pos_R_s = (last_posvel_rx_used_R_us != 0u)
                   ? ((float)((uint32_t)(pos_R_us - last_posvel_rx_used_R_us)) * 1e-6f)
                   : 0.002f;
    last_posvel_rx_used_R_us = pos_R_us;
  }

#if defined(SEND_TORQUE)
  const bool okL = ESC_torque_cmd(sup, can, 0, TORQUE_START_LEFT_NM);
  const bool okR = ESC_torque_cmd(sup, can, 1, TORQUE_START_RIGHT_NM);
  g_can_tx_proof.tx_attempts += 2u;
  g_can_tx_proof.tx_ok += (okL ? 1u : 0u) + (okR ? 1u : 0u);
  g_can_tx_proof.tx_fail += (!okL ? 1u : 0u) + (!okR ? 1u : 0u);
#endif

#if CAN_TX_PROOF_SUMMARY
  const uint32_t now_us = micros();
  if ((uint32_t)(now_us - g_can_tx_proof.last_report_us) >= 1000000u) {
    const uint32_t attempts = g_can_tx_proof.tx_attempts;
    const float fail_pct = (attempts > 0u)
                             ? (100.0f * (float)g_can_tx_proof.tx_fail / (float)attempts)
                             : 0.0f;
    const uint32_t last_posvel_rx_us = canGetLastPosVelRxUs();
    const uint32_t posvel_age_us =
        (last_posvel_rx_us > 0u) ? (uint32_t)(now_us - last_posvel_rx_us) : UINT32_MAX;
    Serial.printf(
        "{\"cmd\":\"CAN_TXQ_SUM\",\"attempts\":%lu,\"ok\":%lu,\"fail\":%lu,\"fail_pct\":%.3f,\"mode\":%d,\"posvel_age_us\":%lu}\r\n",
        (unsigned long)g_can_tx_proof.tx_attempts,
        (unsigned long)g_can_tx_proof.tx_ok,
        (unsigned long)g_can_tx_proof.tx_fail,
        fail_pct,
        (int)sup->mode,
        (unsigned long)posvel_age_us);
    g_can_tx_proof.last_report_us += 1000000u;
  }
#endif

  if (++report_counter >= TELEMETRY_DECIMATE) {
    report_counter = 0;
    extern CAN_message_t g_last_can_msg; // You must define this in CAN_helper.cpp
    Serial.printf(
      "{\"t\":%lu,\"node_id\":%u,\"pos_L_raw\":%.6f,\"vel_L_raw\":%.6f,\"new_pos_L\":%d,\"last_can_id\":0x%08lX,\"last_can_data\":[%u,%u,%u,%u,%u,%u,%u,%u]}\r\n",
      micros(),
      sup->esc[0].config.node_id,
      pos_L,
      vel_L,
      new_pos_L ? 1 : 0,
      (unsigned long)g_last_can_msg.id,
      g_last_can_msg.buf[0], g_last_can_msg.buf[1], g_last_can_msg.buf[2], g_last_can_msg.buf[3],
      g_last_can_msg.buf[4], g_last_can_msg.buf[5], g_last_can_msg.buf[6], g_last_can_msg.buf[7]);
  }

  if (sup->user_total_us > 0 && elapsed_us > sup->user_total_us) {
#ifdef SEND_TORQUE
    const bool okStopL = ESC_torque_cmd(sup, can, 0, 0.0f);
    const bool okStopR = ESC_torque_cmd(sup, can, 1, 0.0f);
    g_can_tx_proof.tx_attempts += 2u;
    g_can_tx_proof.tx_ok += (okStopL ? 1u : 0u) + (okStopR ? 1u : 0u);
    g_can_tx_proof.tx_fail += (!okStopL ? 1u : 0u) + (!okStopR ? 1u : 0u);
#endif

  }
}
