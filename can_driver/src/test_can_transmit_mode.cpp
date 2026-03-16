#include "supervisor.h"
#include "test_can_transmit_mode.h"
#include <Arduino.h>
#include <FlexCAN_T4.h>

#define SEND_TORQUE
#define SEND_TELEMETRY
#define CAN_TX_PROOF_SUMMARY 1

static constexpr uint16_t TEST_ESC_NODE_ID_LEFT = 11u;
static constexpr uint16_t TEST_ESC_NODE_ID_RIGHT = 12u;
static constexpr float TEST_TORQUE_NM = 2.0f;
static constexpr uint32_t TEST_DURATION_US = 1000000u;

static int report_counter = 0;
static bool first_entry = true;
static uint32_t start_time_us = 0;

struct CanTxProofStats {
  uint32_t tx_attempts = 0;
  uint32_t tx_ok = 0;
  uint32_t tx_fail = 0;
  uint32_t last_report_us = 0;
};
static CanTxProofStats g_can_tx_proof;

static bool ESC_torque_cmd_node(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can,
                                uint16_t node_id,
                                float torque) {
  CAN_message_t msg;
  msg.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, node_id);
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

  if (first_entry) {
    first_entry = false;
    start_time_us = micros();
    report_counter = 0;
    g_can_tx_proof = CanTxProofStats{};
    g_can_tx_proof.last_report_us = start_time_us;
    Serial.printf("{\"cmd\":\"PRINT\",\"note\":\"test_can_transmit_mode start\",\"node_l\":%u,\"node_r\":%u,\"torque\":%.3f,\"duration_us\":%lu}\r\n",
                  (unsigned)TEST_ESC_NODE_ID_LEFT,
                  (unsigned)TEST_ESC_NODE_ID_RIGHT,
                  (double)TEST_TORQUE_NM,
                  (unsigned long)TEST_DURATION_US);
  }

  const uint32_t now_us = micros();
  const uint32_t elapsed_us = (uint32_t)(now_us - start_time_us);

#if defined(SEND_TORQUE)
  const bool okL = ESC_torque_cmd_node(can, TEST_ESC_NODE_ID_LEFT, TEST_TORQUE_NM);
  const bool okR = ESC_torque_cmd_node(can, TEST_ESC_NODE_ID_RIGHT, TEST_TORQUE_NM);
  g_can_tx_proof.tx_attempts += 2u;
  g_can_tx_proof.tx_ok += (okL ? 1u : 0u) + (okR ? 1u : 0u);
  g_can_tx_proof.tx_fail += (!okL ? 1u : 0u) + (!okR ? 1u : 0u);
#endif

#if CAN_TX_PROOF_SUMMARY
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
    Serial.printf("{\"cmd\":\"CAN_TX_PROGRESS\",\"elapsed_us\":%lu,\"attempts\":%lu,\"ok\":%lu,\"fail\":%lu}\r\n",
                  (unsigned long)elapsed_us,
                  (unsigned long)g_can_tx_proof.tx_attempts,
                  (unsigned long)g_can_tx_proof.tx_ok,
                  (unsigned long)g_can_tx_proof.tx_fail);
  }

  if (elapsed_us >= TEST_DURATION_US) {
#ifdef SEND_TORQUE
    const bool okStopL = ESC_torque_cmd_node(can, TEST_ESC_NODE_ID_LEFT, 0.0f);
    const bool okStopR = ESC_torque_cmd_node(can, TEST_ESC_NODE_ID_RIGHT, 0.0f);
    g_can_tx_proof.tx_attempts += 2u;
    g_can_tx_proof.tx_ok += (okStopL ? 1u : 0u) + (okStopR ? 1u : 0u);
    g_can_tx_proof.tx_fail += (!okStopL ? 1u : 0u) + (!okStopR ? 1u : 0u);
#endif
    const float fail_pct = (g_can_tx_proof.tx_attempts > 0u)
                             ? (100.0f * (float)g_can_tx_proof.tx_fail / (float)g_can_tx_proof.tx_attempts)
                             : 0.0f;
    Serial.printf("{\"cmd\":\"CAN_TX_DONE\",\"duration_us\":%lu,\"attempts\":%lu,\"ok\":%lu,\"fail\":%lu,\"fail_pct\":%.3f}\r\n",
                  (unsigned long)elapsed_us,
                  (unsigned long)g_can_tx_proof.tx_attempts,
                  (unsigned long)g_can_tx_proof.tx_ok,
                  (unsigned long)g_can_tx_proof.tx_fail,
                  (double)fail_pct);
    Serial.println("button: exiting test_can mode -> idle");

    sup->mode = SUP_MODE_IDLE;
    first_entry = true;
  }
}
