#include <FlexCAN_T4.h>
#include "LED.h"
#include <ArduinoJson.h>
#include <ctype.h>
#include <string.h>
#include "main.h"
#include "pushbutton.h"
#include "tone_player.h"
#include "ESC.h"
#include "CAN_helper.h"
#include "supervisor.h"
#include "test_can_transmit_mode.h"

// ---------------------- Setup / Loop -----------------------
IntervalTimer g_ctrlTimer;
Supervisor_typedef supervisor;

const char* esc_names[]   = {"left", "right"};
const uint16_t esc_ids[]  = {11, 12}; // node_ids of the ESCs
const uint8_t rc_pins[]   = {RC_INPUT1, RC_INPUT2, RC_INPUT3, RC_INPUT4};


// -------------------- CAN Communication --------------------
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;
CANBuffer canRxBuf;  // ✅ holds buffer + link_ok

// -------------------- Tone / Pushbutton --------------------
static TonePlayer g_tone;
PushButton g_button(PUSHBUTTON_PIN, true, 50000u);
static constexpr uint32_t CAN_POSVEL_RX_TIMEOUT_US = 400000u;
static constexpr uint32_t BALANCE_BUTTON_RUN_US = 30000000u;  // 30 seconds
// Runtime decimated printing can perturb timing; keep off for measurement runs.
#define CAN_RUNTIME_DIAG_PRINT 0

// Uncomment to ignore pushbutton state transitions.
#define PB_OVERRIDE
#define CAN_TEST_FW_TAG "teensy_seqdiag_v1"

template <typename TCAN>
static void configure_can_hw_filters(TCAN &can) {
  // Accept only expected extended telemetry frames from node 11 and 12.
  can.setFIFOFilter(REJECT_ALL);
  can.setMBFilter(REJECT_ALL);
  (void)can.setFIFOFilter(0, canMakeExtId(CAN_ID_POSVEL, 11u, 0u), EXT);
  (void)can.setFIFOFilter(1, canMakeExtId(CAN_ID_POSVEL, 12u, 0u), EXT);
  (void)can.setFIFOFilter(2, canMakeExtId(CAN_ID_TEMPS, 11u, 0u), EXT);
  (void)can.setFIFOFilter(3, canMakeExtId(CAN_ID_TEMPS, 12u, 0u), EXT);
}

void beeper_tweet() {
  tone_start(&g_tone, 2800, 60, 0);
}

static void trim_ascii(char *s) {
  if (s == nullptr) return;
  size_t len = strlen(s);
  size_t start = 0;
  while (start < len && isspace((unsigned char)s[start])) start++;
  size_t end = len;
  while (end > start && isspace((unsigned char)s[end - 1])) end--;
  if (start > 0 && end > start) {
    memmove(s, s + start, end - start);
  }
  s[end - start] = '\0';
}

static void process_serial_line(const char *line) {
  if (line == nullptr) return;

  if (strcmp(line, "run") == 0) {
    tone_start(&g_tone, PB_BEEP_HZ, PB_BEEP_MS, PB_GAP_MS);
    canResetPosvelStats();
    canResetRuntimeStats();
    resetControlDtStats();
    canRxBuf.overflow_count = 0;
    for (uint16_t i = 0; i < supervisor.esc_count; ++i) {
      supervisor.esc_alive_false_count[i] = 0u;
    }
    supervisor.user_total_us = BALANCE_BUTTON_RUN_US;
    supervisor.mode = SUP_MODE_TEST_CAN;
    return;
  }

  if (strcmp(line, "tx off") == 0) {
    supervisor.user_tx_enable = false;
    Serial.printf("{\"cmd\":\"CAN_TX_CFG\",\"tx_enable\":0,\"tx_period_us\":%lu,\"tx_hz\":%.2f}\r\n",
                  (unsigned long)supervisor.user_tx_period_us,
                  (supervisor.user_tx_period_us > 0u)
                    ? (1000000.0f / (float)supervisor.user_tx_period_us)
                    : 0.0f);
    return;
  }

  if (strcmp(line, "tx on") == 0) {
    supervisor.user_tx_enable = true;
    Serial.printf("{\"cmd\":\"CAN_TX_CFG\",\"tx_enable\":1,\"tx_period_us\":%lu,\"tx_hz\":%.2f}\r\n",
                  (unsigned long)supervisor.user_tx_period_us,
                  (supervisor.user_tx_period_us > 0u)
                    ? (1000000.0f / (float)supervisor.user_tx_period_us)
                    : 0.0f);
    return;
  }

  if (strcmp(line, "stats reset") == 0) {
    canResetPosvelStats();
    canResetRuntimeStats();
    resetControlDtStats();
    canRxBuf.overflow_count = 0;
    for (uint16_t i = 0; i < supervisor.esc_count; ++i) {
      supervisor.esc_alive_false_count[i] = 0u;
    }
    Serial.printf("{\"cmd\":\"CAN_STATS_RESET\",\"ok\":1}\r\n");
    return;
  }

  uint32_t hz = 0;
  if (sscanf(line, "tx hz %lu", &hz) == 1) {
    if (hz == 0u || hz > 2000u) {
      Serial.printf("{\"cmd\":\"CAN_TX_CFG_ERR\",\"reason\":\"hz_out_of_range\",\"hz\":%lu,\"min\":1,\"max\":2000}\r\n",
                    (unsigned long)hz);
    } else {
      supervisor.user_tx_period_us = 1000000u / hz;
      if (supervisor.user_tx_period_us == 0u) supervisor.user_tx_period_us = 1u;
      Serial.printf("{\"cmd\":\"CAN_TX_CFG\",\"tx_enable\":%d,\"tx_period_us\":%lu,\"tx_hz\":%.2f}\r\n",
                    supervisor.user_tx_enable ? 1 : 0,
                    (unsigned long)supervisor.user_tx_period_us,
                    (supervisor.user_tx_period_us > 0u)
                      ? (1000000.0f / (float)supervisor.user_tx_period_us)
                      : 0.0f);
    }
    return;
  }

  Serial.printf("{\"cmd\":\"CAN_CMD_ERR\",\"line\":\"%s\"}\r\n", line);
}

// --------------------- LED instances -----------------------
static LEDCtrl g_led_red;
LEDCtrl g_led_green;

void setup() {

  Serial.begin(921600);
  while (!Serial && millis() < 1500) {}
  Serial.printf("{\"cmd\":\"FW_TAG\",\"tag\":\"%s\"}\r\n", CAN_TEST_FW_TAG);
  // CAN2 uses pins 0/1 on Teensy 4.0, so avoid Serial1 on those same pins.

  // LEDs / Pushbutton / Tone
  led_init(&g_led_red,   LED1_PIN, LED_BLINK_SLOW);
  led_init(&g_led_green, LED2_PIN, LED_BLINK_FAST);
  tone_init(&g_tone, SPEAKER_PIN);

  // ---- CAN Setup ----
  // CAN1 default routing on Teensy 4.0: RX=pin 23, TX=pin 22.
#if CAN1_ENABLE
  Can1.setRX(CAN1_PINSEL);
  Can1.setTX(CAN1_PINSEL);
  Can1.begin();
  Can1.setBaudRate(500000);
  Can1.enableFIFO();
  configure_can_hw_filters(Can1);
#endif

  // CAN2 default routing on Teensy 4.0: RX=pin 0, TX=pin 1.
  Can2.setRX(CAN2_PINSEL);
  Can2.setTX(CAN2_PINSEL);
  Can2.begin();
  Can2.setBaudRate(500000);
  Can2.enableFIFO();
  configure_can_hw_filters(Can2);
  pinMode(CAN_STB, OUTPUT);
  digitalWrite(CAN_STB, LOW);

  init_supervisor(&supervisor,
                  2,           // esc_count -- FIX: dont hard code this number
                  esc_names,   // ESC names
                  esc_ids,     // ESC node IDs
                  rc_pins,     // RC pins
                  4);          // RC count -- FIX: dont hard code this number

  // Start in idle; require explicit serial command "run" to begin test mode.
  supervisor.mode = SUP_MODE_IDLE;
  supervisor.user_total_us = 0;
  supervisor.user_tx_enable = true;
  supervisor.user_tx_period_us = 1000;  // default 1 kHz command TX
  canResetPosvelStats();
  canResetRuntimeStats();
  canRxBuf.overflow_count = 0;

  // ---- Control tick ISR ----
  g_ctrlTimer.priority(CONTROL_LOOP_PRIORITY);
  g_ctrlTimer.begin(controlLoop_isr, CONTROL_PERIOD_US);

}

void loop() {
  // -------- HIGH PRIORITY --------
  // Set to CONTROL_PERIOD_US = 1000 µs (1000 Hz).
  // NOTE: ESC POSVEL over CAN is expected around 500 Hz (about every 2 ms),
  // while this control loop runs at 1 kHz (every 1 ms). They are intentionally
  // asynchronous, so many control ticks will reuse the latest POSVEL sample.
  // The system uses an ISR-driven scheduler to tell the main loop to go into controlLoop. 
  // ---
  while (true) {
    noInterrupts();
    const uint32_t pending = g_control_pending_ticks;
    if (pending == 0u) {
      interrupts();
      break;
    }
    g_control_pending_ticks = pending - 1u;
    interrupts();
    controlLoop(&supervisor, Can1, Can2);
  }

  // -------- CAN POLLING --------
  // Non-blocking and not based on an ISR because FLEXCAN_T4 did seem to work. 
  // ---
  CAN_message_t msg;
#if CAN1_ENABLE
  while (Can1.read(msg)) {
    canNoteBusRead(1u);
    if (!canBufferPush(canRxBuf, msg, 1u)) {
      canNoteRxOverflow();
    }
  }
#endif
  while (Can2.read(msg)) {
    canNoteBusRead(2u);
    if (!canBufferPush(canRxBuf, msg, 2u)) {
      canNoteRxOverflow();
    }
  }
  uint8_t rx_bus = 0u;
  while (canBufferPop(canRxBuf, msg, &rx_bus)) {
    handleCANMessage(msg, rx_bus);
  }

  uint32_t now_us = micros();
  uint32_t last_posvel_rx_us = canGetLastPosVelRxUs();
  bool posvel_rx_fresh = (last_posvel_rx_us != 0u) &&
                         ((uint32_t)(now_us - last_posvel_rx_us) <= CAN_POSVEL_RX_TIMEOUT_US);
  led_set_state(&g_led_red, posvel_rx_fresh ? LED_PULSE : LED_OFF);
  led_update(&g_led_red, now_us);

  // PUSHBUTTON (run every loop to avoid latency in target capture)
#ifndef PB_OVERRIDE
  g_button.update(now_us);
  if (g_button.hasChanged()) {
    PBState pb_state = g_button.getState();

    if (pb_state == PB_PRESSED) {
      tone_start(&g_tone, PB_BEEP_HZ, PB_BEEP_MS, PB_GAP_MS);
    }
	    else if (pb_state == PB_RELEASED) {
	      SupervisorMode test_mode = SUP_MODE_TEST_CAN;
	      if (supervisor.mode != test_mode) {
          canResetPosvelStats();
          canResetRuntimeStats();
          resetControlDtStats();
          canRxBuf.overflow_count = 0;
          for (uint16_t i = 0; i < supervisor.esc_count; ++i) {
            supervisor.esc_alive_false_count[i] = 0u;
          }
	        supervisor.user_total_us = BALANCE_BUTTON_RUN_US;
	        supervisor.mode = test_mode;
	      }
	    }
    g_button.clearChanged();
  }
#endif

  static char input_buf[96] = {0};
  static size_t input_len = 0;

  // -------- MEDIUM PRIORITY --------
  // Non-RT, faster than low-priority tasks so dump transmission can drain quickly.
  static uint32_t last_medprio_us = 0;
  if (now_us - last_medprio_us >= (CONTROL_PERIOD_US * 10)) {  // 10 ms at 1 kHz base
    last_medprio_us = now_us;

    // Live CAN RX health printing intentionally disabled during timing tests.
#if CAN_RUNTIME_DIAG_PRINT
    static uint32_t last_can_diag_us = 0;
    if (supervisor.mode == SUP_MODE_TEST_CAN &&
        (uint32_t)(now_us - last_can_diag_us) >= 1000000u) {
      last_can_diag_us = now_us;

      PosvelRxStats left{};
      PosvelRxStats right{};
      PosvelRxStats can1_bus{};
      PosvelRxStats can2_bus{};
      const CanRuntimeStats rt = canGetRuntimeStats();
      const uint8_t left_id = supervisor.esc[0].config.node_id;
      const uint8_t right_id = supervisor.esc[1].config.node_id;
      const bool have_left = canGetPosvelRxStats(left_id, left);
      const bool have_right = canGetPosvelRxStats(right_id, right);
      const bool have_can1_bus = canGetBusPosvelRxStats(1u, can1_bus);
      const bool have_can2_bus = canGetBusPosvelRxStats(2u, can2_bus);

      const uint32_t left_age_us =
          (have_left && left.last_rx_us > 0u) ? (uint32_t)(now_us - left.last_rx_us) : UINT32_MAX;
      const uint32_t right_age_us =
          (have_right && right.last_rx_us > 0u) ? (uint32_t)(now_us - right.last_rx_us) : UINT32_MAX;
      const uint32_t left_avg_gap_us =
          (have_left && left.gap_count > 0u) ? (uint32_t)(left.sum_gap_us / left.gap_count) : 0u;
      const uint32_t right_avg_gap_us =
          (have_right && right.gap_count > 0u) ? (uint32_t)(right.sum_gap_us / right.gap_count) : 0u;
      const uint32_t left_min_gap_us =
          (have_left && left.gap_count > 0u) ? left.min_gap_us : 0u;
      const uint32_t right_min_gap_us =
          (have_right && right.gap_count > 0u) ? right.min_gap_us : 0u;
      const uint32_t left_p50_gap_us =
          (have_left && left.gap_count > 0u) ? left.p50_gap_us : 0u;
      const uint32_t right_p50_gap_us =
          (have_right && right.gap_count > 0u) ? right.p50_gap_us : 0u;
      const uint32_t left_p95_gap_us =
          (have_left && left.gap_count > 0u) ? left.p95_gap_us : 0u;
      const uint32_t right_p95_gap_us =
          (have_right && right.gap_count > 0u) ? right.p95_gap_us : 0u;
      const uint32_t left_p99_gap_us =
          (have_left && left.gap_count > 0u) ? left.p99_gap_us : 0u;
      const uint32_t right_p99_gap_us =
          (have_right && right.gap_count > 0u) ? right.p99_gap_us : 0u;
      const uint32_t can1_age_us =
          (have_can1_bus && can1_bus.last_rx_us > 0u) ? (uint32_t)(now_us - can1_bus.last_rx_us) : UINT32_MAX;
      const uint32_t can2_age_us =
          (have_can2_bus && can2_bus.last_rx_us > 0u) ? (uint32_t)(now_us - can2_bus.last_rx_us) : UINT32_MAX;
      const uint32_t can1_avg_gap_us =
          (have_can1_bus && can1_bus.gap_count > 0u) ? (uint32_t)(can1_bus.sum_gap_us / can1_bus.gap_count) : 0u;
      const uint32_t can2_avg_gap_us =
          (have_can2_bus && can2_bus.gap_count > 0u) ? (uint32_t)(can2_bus.sum_gap_us / can2_bus.gap_count) : 0u;
      const uint32_t can1_p50_gap_us =
          (have_can1_bus && can1_bus.gap_count > 0u) ? can1_bus.p50_gap_us : 0u;
      const uint32_t can2_p50_gap_us =
          (have_can2_bus && can2_bus.gap_count > 0u) ? can2_bus.p50_gap_us : 0u;
      const uint32_t can1_p95_gap_us =
          (have_can1_bus && can1_bus.gap_count > 0u) ? can1_bus.p95_gap_us : 0u;
      const uint32_t can2_p95_gap_us =
          (have_can2_bus && can2_bus.gap_count > 0u) ? can2_bus.p95_gap_us : 0u;
      const uint32_t can1_p99_gap_us =
          (have_can1_bus && can1_bus.gap_count > 0u) ? can1_bus.p99_gap_us : 0u;
      const uint32_t can2_p99_gap_us =
          (have_can2_bus && can2_bus.gap_count > 0u) ? can2_bus.p99_gap_us : 0u;
      const int32_t lr_offset_us =
          (have_left && have_right && left.last_rx_us > 0u && right.last_rx_us > 0u)
              ? (int32_t)(right.last_rx_us - left.last_rx_us)
              : INT32_MAX;
      const uint32_t lr_offset_abs_us =
          (lr_offset_us == INT32_MAX)
              ? UINT32_MAX
              : (uint32_t)((lr_offset_us < 0) ? -lr_offset_us : lr_offset_us);
      const int32_t lr_count_delta =
          (have_left && have_right)
              ? (int32_t)right.count - (int32_t)left.count
              : INT32_MAX;

      Serial.printf(
          "{\"cmd\":\"CAN_POSVEL_RX\",\"t\":%lu,"
          "\"left_id\":%u,\"left_count\":%lu,\"left_age_us\":%lu,"
          "\"left_avg_gap_us\":%lu,\"left_min_gap_us\":%lu,\"left_p50_gap_us\":%lu,\"left_p95_gap_us\":%lu,\"left_p99_gap_us\":%lu,\"left_max_gap_us\":%lu,\"left_est_missed\":%lu,"
          "\"right_id\":%u,\"right_count\":%lu,\"right_age_us\":%lu,"
          "\"right_avg_gap_us\":%lu,\"right_min_gap_us\":%lu,\"right_p50_gap_us\":%lu,\"right_p95_gap_us\":%lu,\"right_p99_gap_us\":%lu,\"right_max_gap_us\":%lu,\"right_est_missed\":%lu,"
          "\"can1_posvel_count\":%lu,\"can1_posvel_age_us\":%lu,\"can1_posvel_avg_gap_us\":%lu,\"can1_posvel_p50_gap_us\":%lu,\"can1_posvel_p95_gap_us\":%lu,\"can1_posvel_p99_gap_us\":%lu,"
          "\"can2_posvel_count\":%lu,\"can2_posvel_age_us\":%lu,\"can2_posvel_avg_gap_us\":%lu,\"can2_posvel_p50_gap_us\":%lu,\"can2_posvel_p95_gap_us\":%lu,\"can2_posvel_p99_gap_us\":%lu,"
          "\"can1_rx_reads\":%lu,\"can2_rx_reads\":%lu,\"rx_overflow\":%lu,"
          "\"lr_offset_us\":%ld,\"lr_offset_abs_us\":%lu,\"lr_count_delta\":%ld}\r\n",
          (unsigned long)now_us,
          left_id,
          (unsigned long)(have_left ? left.count : 0u),
          (unsigned long)left_age_us,
          (unsigned long)left_avg_gap_us,
          (unsigned long)left_min_gap_us,
          (unsigned long)left_p50_gap_us,
          (unsigned long)left_p95_gap_us,
          (unsigned long)left_p99_gap_us,
          (unsigned long)(have_left ? left.max_gap_us : 0u),
          (unsigned long)(have_left ? left.est_missed : 0u),
          right_id,
          (unsigned long)(have_right ? right.count : 0u),
          (unsigned long)right_age_us,
          (unsigned long)right_avg_gap_us,
          (unsigned long)right_min_gap_us,
          (unsigned long)right_p50_gap_us,
          (unsigned long)right_p95_gap_us,
          (unsigned long)right_p99_gap_us,
          (unsigned long)(have_right ? right.max_gap_us : 0u),
          (unsigned long)(have_right ? right.est_missed : 0u),
          (unsigned long)(have_can1_bus ? can1_bus.count : 0u),
          (unsigned long)can1_age_us,
          (unsigned long)can1_avg_gap_us,
          (unsigned long)can1_p50_gap_us,
          (unsigned long)can1_p95_gap_us,
          (unsigned long)can1_p99_gap_us,
          (unsigned long)(have_can2_bus ? can2_bus.count : 0u),
          (unsigned long)can2_age_us,
          (unsigned long)can2_avg_gap_us,
          (unsigned long)can2_p50_gap_us,
          (unsigned long)can2_p95_gap_us,
          (unsigned long)can2_p99_gap_us,
          (unsigned long)rt.can1_rx_reads,
          (unsigned long)rt.can2_rx_reads,
          (unsigned long)rt.rx_overflow,
          (long)lr_offset_us,
          (unsigned long)lr_offset_abs_us,
          (long)lr_count_delta);
    }
#endif
  }

  // -------- LOW PRIORITY --------
  // These functions are intentionally throttled and run infrequently.
  // ---

  static uint32_t last_lowprio_us = 0;

  if (now_us - last_lowprio_us >= (CONTROL_PERIOD_US * 100)) {
    last_lowprio_us = now_us;

	    while (Serial.available()) {
	      char c = Serial.read();
        Serial.write((uint8_t)c);  // Echo input characters back to terminal.
	      if (c == '\r' || c == '\n') {
          if (input_len > 0u) {
            input_buf[input_len] = '\0';
            trim_ascii(input_buf);
            if (input_buf[0] != '\0') {
              process_serial_line(input_buf);
            }
            input_len = 0u;
            input_buf[0] = '\0';
          }
	      } else if (c == '\b' || c == 127) {
          if (input_len > 0u) {
            input_len--;
            input_buf[input_len] = '\0';
          }
        } else if (isprint((unsigned char)c)) {
          if (input_len < (sizeof(input_buf) - 1u)) {
            input_buf[input_len++] = c;
            input_buf[input_len] = '\0';
          }
        }
	    }

    // LED CONTROL
    tone_update(&g_tone, now_us);
    led_update(&g_led_green, now_us);

    // 1 Hz HEALTH CHECK
    if (millis() - supervisor.last_health_ms > 1000) {
      supervisor.last_health_ms = millis();
      led_set_state(&g_led_green, canRxBuf.link_ok ? LED_ON_CONTINUOUS : LED_BLINK_SLOW);
    }

    // Resets timing stats
    if (supervisor.timing.count > 0) { resetLoopTimingStats(&supervisor);  }
  } // end of low priority loop
}
