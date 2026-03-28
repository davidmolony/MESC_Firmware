#include "supervisor.h"
#include "test_can_transmit_mode.h"
#include <Arduino.h>
#include <FlexCAN_T4.h>

#define CAN_TX_PROOF_SUMMARY 1

static constexpr float TORQUE_START_LEFT_NM = 1.0f;
static constexpr float TORQUE_START_RIGHT_NM = -1.0f;

static bool first_entry = true;
static uint32_t start_time_us = 0;
static uint32_t last_tx_us = 0;

struct CanTxProofStats {
  uint32_t tx_attempts = 0;
  uint32_t tx_ok = 0;
  uint32_t tx_fail = 0;
  uint32_t last_report_us = 0;
};
static CanTxProofStats g_can_tx_proof;

static bool esc_is_alive(const Supervisor_typedef *sup, uint8_t esc_num) {
  return (sup != nullptr) && (esc_num < sup->esc_count) && sup->esc[esc_num].state.alive;
}

static bool ESC_torque_cmd(Supervisor_typedef *sup,
                           FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can1,
                           FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> &can2,
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
  const uint8_t tx_bus = can_tx_bus_for_node(sup->esc[esc_num].config.node_id);
#if CAN1_ENABLE
  const bool ok1 = (tx_bus == CAN_TX_BUS_CAN1 || tx_bus == CAN_TX_BUS_BOTH)
                     ? can1.write(msg)
                     : false;
#else
  const bool ok1 = false;
#endif
  const bool ok2 = (tx_bus == CAN_TX_BUS_CAN2 || tx_bus == CAN_TX_BUS_BOTH)
                     ? can2.write(msg)
                     : false;
  return ok1 || ok2;
}

void test_can_transmit_mode(Supervisor_typedef *sup,
                            FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can1,
                            FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> &can2) {
  if (!sup) return;
  const bool aliveL = esc_is_alive(sup, 0);
  const bool aliveR = esc_is_alive(sup, 1);

  if (!aliveL && !aliveR) {
    if (sup->esc_count > 0u) {
      ESC_torque_cmd(sup, can1, can2, 0, 0.0f);
    }
    if (sup->esc_count > 1u) {
      ESC_torque_cmd(sup, can1, can2, 1, 0.0f);
    }

    sup->mode = SUP_MODE_IDLE;
    first_entry = true;
    last_tx_us = 0;
    return;
  }

  if (first_entry) {
    first_entry = false;
    start_time_us = micros();
    last_tx_us = 0;
    g_can_tx_proof = CanTxProofStats{};
    g_can_tx_proof.last_report_us = start_time_us;
  }

  const uint32_t now_us = micros();
  const uint32_t elapsed_us = now_us - start_time_us;
  const uint32_t tx_period_us = (sup->user_tx_period_us > 0u) ? sup->user_tx_period_us : 1000u;
  const bool tx_due = (last_tx_us == 0u) || ((uint32_t)(now_us - last_tx_us) >= tx_period_us);

  if (sup->user_tx_enable && tx_due) {
    last_tx_us = now_us;
    if (aliveL) {
      const bool okL = ESC_torque_cmd(sup, can1, can2, 0, TORQUE_START_LEFT_NM);
      g_can_tx_proof.tx_attempts += 1u;
      g_can_tx_proof.tx_ok += okL ? 1u : 0u;
      g_can_tx_proof.tx_fail += okL ? 0u : 1u;
    }
    if (aliveR) {
      const bool okR = ESC_torque_cmd(sup, can1, can2, 1, TORQUE_START_RIGHT_NM);
      g_can_tx_proof.tx_attempts += 1u;
      g_can_tx_proof.tx_ok += okR ? 1u : 0u;
      g_can_tx_proof.tx_fail += okR ? 0u : 1u;
    }
  }

#if CAN_TX_PROOF_SUMMARY
  if ((uint32_t)(now_us - g_can_tx_proof.last_report_us) >= 1000000u) {
    const uint32_t attempts = g_can_tx_proof.tx_attempts;
    const float fail_pct = (attempts > 0u)
                             ? (100.0f * (float)g_can_tx_proof.tx_fail / (float)attempts)
                             : 0.0f;
    const CanRuntimeStats rt = canGetRuntimeStats();
    const uint32_t last_posvel_rx_us = canGetLastPosVelRxUs();
    const uint32_t posvel_age_us =
        (last_posvel_rx_us > 0u) ? (uint32_t)(now_us - last_posvel_rx_us) : UINT32_MAX;
    const float tx_hz = (tx_period_us > 0u) ? (1000000.0f / (float)tx_period_us) : 0.0f;
    Serial.printf(
        "{\"cmd\":\"CAN_TXQ_SUM\",\"attempts\":%lu,\"ok\":%lu,\"fail\":%lu,\"fail_pct\":%.3f,\"mode\":%d,\"tx_enable\":%d,\"tx_period_us\":%lu,\"tx_hz\":%.2f,\"posvel_age_us\":%lu,\"can1_rx_reads\":%lu,\"can2_rx_reads\":%lu,\"rx_overflow\":%lu}\r\n",
        (unsigned long)g_can_tx_proof.tx_attempts,
        (unsigned long)g_can_tx_proof.tx_ok,
        (unsigned long)g_can_tx_proof.tx_fail,
        fail_pct,
        (int)sup->mode,
        sup->user_tx_enable ? 1 : 0,
        (unsigned long)tx_period_us,
        tx_hz,
        (unsigned long)posvel_age_us,
        (unsigned long)rt.can1_rx_reads,
        (unsigned long)rt.can2_rx_reads,
        (unsigned long)rt.rx_overflow);
    g_can_tx_proof.last_report_us += 1000000u;
  }
#endif

  if (sup->user_total_us > 0 && elapsed_us > sup->user_total_us) {
    if (sup->esc_count > 0u) {
      const bool okStopL = ESC_torque_cmd(sup, can1, can2, 0, 0.0f);
      g_can_tx_proof.tx_attempts += 1u;
      g_can_tx_proof.tx_ok += okStopL ? 1u : 0u;
      g_can_tx_proof.tx_fail += okStopL ? 0u : 1u;
    }
    if (sup->esc_count > 1u) {
      const bool okStopR = ESC_torque_cmd(sup, can1, can2, 1, 0.0f);
      g_can_tx_proof.tx_attempts += 1u;
      g_can_tx_proof.tx_ok += okStopR ? 1u : 0u;
      g_can_tx_proof.tx_fail += okStopR ? 0u : 1u;
    }
    Serial.printf(
      "{\"cmd\":\"CAN_TX_DONE\",\"elapsed_us\":%lu,\"limit_us\":%lu,\"tx_fail\":%lu}\r\n",
      (unsigned long)elapsed_us,
      (unsigned long)sup->user_total_us,
      (unsigned long)g_can_tx_proof.tx_fail);

    // End this run deterministically and wait for the next explicit "run" command.
    sup->mode = SUP_MODE_IDLE;
    sup->user_total_us = 0;
    first_entry = true;
    last_tx_us = 0;
  }
}
