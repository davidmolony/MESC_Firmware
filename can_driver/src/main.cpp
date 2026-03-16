#include <FlexCAN_T4.h>
#include "LED.h"
#include <ArduinoJson.h>
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
CANBuffer canRxBuf;  // ✅ holds buffer + link_ok

// -------------------- Tone / Pushbutton --------------------
static TonePlayer g_tone;
PushButton g_button(PUSHBUTTON_PIN, true, 50000u);
static constexpr uint32_t CAN_POSVEL_RX_TIMEOUT_US = 400000u;
static constexpr uint32_t BALANCE_BUTTON_RUN_US = 1000000u;  // 1 second

// Uncomment to ignore pushbutton state transitions.
// #define PB_OVERRIDE

void beeper_tweet() {
  tone_start(&g_tone, 2800, 60, 0);
}

// --------------------- LED instances -----------------------
static LEDCtrl g_led_red;
LEDCtrl g_led_green;

void setup() {

  Serial.begin(921600);
  while (!Serial && millis() < 1500) {}
  Serial1.begin(115200);  // Teensy TX1/RX1 -> ESP32 RX/TX, 8N1, no flow control.

  // LEDs / Pushbutton / Tone
  led_init(&g_led_red,   LED1_PIN, LED_BLINK_SLOW);
  led_init(&g_led_green, LED2_PIN, LED_BLINK_FAST);
  tone_init(&g_tone, SPEAKER_PIN);

  // ---- CAN Setup ----
  Can1.begin();
  Can1.setBaudRate(500000);
  Can1.enableFIFO();
  pinMode(CAN_STB, OUTPUT);
  digitalWrite(CAN_STB, LOW);

  init_supervisor(&supervisor,
                  2,           // esc_count -- FIX: dont hard code this number
                  esc_names,   // ESC names
                  esc_ids,     // ESC node IDs
                  rc_pins,     // RC pins
                  4);          // RC count -- FIX: dont hard code this number

  // Start in IDLE. User button or serial command enters test CAN mode.
  supervisor.mode = SUP_MODE_IDLE;

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
  if (g_control_due) {
    controlLoop(&supervisor, Can1);
    g_control_due = false;
  }

  // -------- CAN POLLING --------
  // Non-blocking and not based on an ISR because FLEXCAN_T4 did seem to work. 
  // ---
  CAN_message_t msg;
  while (Can1.read(msg)) {
    canBufferPush(canRxBuf, msg);
  }
  while (canBufferPop(canRxBuf, msg)) {
    handleCANMessage(msg);
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
      if (supervisor.mode == test_mode) {
        Serial.println("button: test_can mode already active; ignoring");
      } else {
        Serial.printf("button: entering test_can mode (duration_us=%lu)\r\n",
                      (unsigned long)BALANCE_BUTTON_RUN_US);
        supervisor.user_total_us = BALANCE_BUTTON_RUN_US;
        supervisor.mode = test_mode;
      }
    }
    g_button.clearChanged();
  }
#endif

  static String input = "";

  // -------- MEDIUM PRIORITY --------
  // Non-RT, faster than low-priority tasks so dump transmission can drain quickly.
  static uint32_t last_medprio_us = 0;
  if (now_us - last_medprio_us >= (CONTROL_PERIOD_US * 10)) {  // 10 ms at 1 kHz base
    last_medprio_us = now_us;

    // Live CAN RX health for ESP32/python during balance.
    static uint32_t last_can_diag_us = 0;
    if (supervisor.mode == SUP_MODE_TEST_CAN &&
        (uint32_t)(now_us - last_can_diag_us) >= 1000000u) {
      last_can_diag_us = now_us;

      PosvelRxStats left{};
      PosvelRxStats right{};
      const uint8_t left_id = supervisor.esc[0].config.node_id;
      const uint8_t right_id = supervisor.esc[1].config.node_id;
      const bool have_left = canGetPosvelRxStats(left_id, left);
      const bool have_right = canGetPosvelRxStats(right_id, right);

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
        const bool counter_mode = canGetPosvelCounterMode();

        char can_diag[512];
        snprintf(
          can_diag,
          sizeof(can_diag),
          "{\"cmd\":\"CAN_POSVEL_RX\",\"t\":%lu,"
          "\"counter_mode\":%u,"
          "\"left_id\":%u,\"left_count\":%lu,\"left_age_us\":%lu,"
          "\"left_avg_gap_us\":%lu,\"left_min_gap_us\":%lu,\"left_max_gap_us\":%lu,\"left_est_missed\":%lu,\"left_ctr_dup\":%lu,\"left_ctr_jump\":%lu,\"left_ctr_missed\":%lu,"
          "\"right_id\":%u,\"right_count\":%lu,\"right_age_us\":%lu,"
          "\"right_avg_gap_us\":%lu,\"right_min_gap_us\":%lu,\"right_max_gap_us\":%lu,\"right_est_missed\":%lu,\"right_ctr_dup\":%lu,\"right_ctr_jump\":%lu,\"right_ctr_missed\":%lu}\r\n",
          (unsigned long)now_us,
          (unsigned)counter_mode,
          left_id,
          (unsigned long)(have_left ? left.count : 0u),
          (unsigned long)left_age_us,
          (unsigned long)left_avg_gap_us,
          (unsigned long)left_min_gap_us,
          (unsigned long)(have_left ? left.max_gap_us : 0u),
          (unsigned long)(have_left ? left.est_missed : 0u),
          (unsigned long)(have_left ? left.counter_duplicates : 0u),
          (unsigned long)(have_left ? left.counter_jumps : 0u),
          (unsigned long)(have_left ? left.counter_jump_total : 0u),
          right_id,
          (unsigned long)(have_right ? right.count : 0u),
          (unsigned long)right_age_us,
          (unsigned long)right_avg_gap_us,
          (unsigned long)right_min_gap_us,
          (unsigned long)(have_right ? right.max_gap_us : 0u),
          (unsigned long)(have_right ? right.est_missed : 0u),
          (unsigned long)(have_right ? right.counter_duplicates : 0u),
          (unsigned long)(have_right ? right.counter_jumps : 0u),
          (unsigned long)(have_right ? right.counter_jump_total : 0u));

        Serial.print(can_diag);
    }
  }

  // -------- LOW PRIORITY --------
  // These functions are intentionally throttled and run infrequently.
  // ---

  static uint32_t last_lowprio_us = 0;

  if (now_us - last_lowprio_us >= (CONTROL_PERIOD_US * 100)) {
    last_lowprio_us = now_us;

	    while (Serial.available()) {
	      char c = Serial.read();
        Serial.write((uint8_t)c);
	      if (c == '\r' || c == '\n') {
		String line = input;
		line.trim();

		if (line == "run") {
		  Serial.println("serial: received run command; starting test_can_transmit mode");
		  tone_start(&g_tone, PB_BEEP_HZ, PB_BEEP_MS, PB_GAP_MS);
		  supervisor.user_total_us = BALANCE_BUTTON_RUN_US;
		  supervisor.mode = SUP_MODE_TEST_CAN;
    } else if (line == "posvel_counter on") {
      canSetPosvelCounterMode(true);
      Serial.println("serial: posvel counter mode ON");
    } else if (line == "posvel_counter off") {
      canSetPosvelCounterMode(false);
      Serial.println("serial: posvel counter mode OFF");
    } else if (line == "posvel_counter") {
      Serial.printf("serial: posvel counter mode=%u\r\n", (unsigned)(canGetPosvelCounterMode() ? 1u : 0u));
    }
		input = "";  // reset buffer
	      } else {
		input += c;  // append char to buffer
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
