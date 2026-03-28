#ifndef SUPERVISOR_H
#define SUPERVISOR_H

#include <Arduino.h>
#include "ESC.h"
#include <FlexCAN_T4.h>
#include "CAN_helper.h"
#include "main.h"

#define TELEMETRY_DECIMATE 100

#define SUPERVISOR_MAX_ESCS   4
#define RC_INPUT_MAX_PINS     8
#define RC_INPUT_MIN_US       1000
#define RC_INPUT_MAX_US       2000
#define RC_INPUT_TIMEOUT_US   100000 // 100 ms

#define CONTROL_LOOP_PRIORITY 16
#define CONTROL_PERIOD_US     1000   // 1 kHz

// ---------------- Defaults ----------------
static constexpr uint32_t DEFAULT_PULSE_US     = 0;
static constexpr uint32_t DEFAULT_TOTAL_US     = 0;
static constexpr float    DEFAULT_KD_TERM = 0.0f;
static constexpr float    DEFAULT_KP_TERM = 0.0f;
static constexpr float    DEFAULT_PULSE_TORQUE = 0.0f;

// ---------------- Loop Timing Stats ----------------
// Captures jitter and execution time statistics of the control loop.
struct LoopTimingStats {
  uint32_t last_tick_us;   // Timestamp of last loop tick
  uint32_t dt_us;          // Time between consecutive loop ticks
  uint32_t exec_time_us;   // Execution time of last loop

  uint32_t min_dt_us;      // Minimum observed loop period
  uint32_t max_dt_us;      // Maximum observed loop period
  uint64_t sum_dt_us;      // Sum of loop periods (for averaging)
  uint32_t count;          // Number of loop samples collected

  uint32_t overruns;       // Number of times loop exceeded CONTROL_PERIOD_US
};

// ---------------- Telemetry Stats ----------------
// Measures blocking time when writing telemetry over Serial1.
struct SerialStats {
  uint32_t last_block_us;   // Duration of the most recent blocking write
  uint32_t max_block_us;    // Longest blocking time observed
  uint64_t sum_block_us;    // Accumulated total of all blocking times
  uint32_t count;           // Number of writes measured
};

// ---------------- RC Input ----------------
// Raw RC signal input (pulse width in microseconds).
struct RCInputRaw {
  volatile uint16_t raw_us;     // Most recent raw PWM input
  volatile uint32_t last_update;// Timestamp of last update (µs)
};

// Normalized RC channel data after processing.
struct RCChannel {
  float norm;       // Normalized value [-1.0, 1.0] or [0, 1] depending on mapping
  uint16_t raw_us;  // Raw pulse width in µs
  bool valid;       // True if the channel is valid and updated recently
};

// ---------------- Supervisor Modes ----------------
// High-level supervisor state machine.
enum SupervisorMode {
  SUP_MODE_IDLE = 0,   // System idle, no active control
  SUP_MODE_ACTIVE,     // Normal operation / balancing
  SUP_MODE_SINUSOIDAL,
  SUP_MODE_TORQUE_RESPONSE,
  SUP_MODE_BALANCE_TWR,
  SUP_MODE_TEST_CAN,
  SUP_MODE_SET_POSITION,
  SUP_MODE_SIN_TORQUE,
  SUP_MODE_FAULT       // Fault state (error, timeout, etc.)
};

// Locomotion gait mode (future extension).
enum GaitMode {
  GAIT_IDLE = 0,       // Standing still
  GAIT_WALK,           // Walking gait
  GAIT_RUN             // Running gait
};

// ---------------- Supervisor ----------------
// Central state container for the system.
// Holds ESC state, IMU, RC input, timing stats, telemetry stats, etc.
struct Supervisor_typedef {
  uint16_t       esc_count;                                // Number of ESCs managed
  ESC            esc[SUPERVISOR_MAX_ESCS];                 // Array of ESC objects
  uint32_t       last_esc_heartbeat_us[SUPERVISOR_MAX_ESCS]; // Last CAN heartbeat timestamps per ESC

  SupervisorMode mode;                                     // Current supervisor mode
  GaitMode       gait_mode;                                // Current gait mode

  float user_setpoint = M_PI;
  float user_Kp_term = 0.0f;
  float user_Kd_term = 0.0f; 
  float user_pulse_torque   = 0.2f;    // amplitude of torque pulse
  uint32_t user_pulse_us    = 1000;   // duration of pulse (µs)
  uint32_t user_total_us    = 1000;
  bool user_tx_enable       = true;   // Enables/disables torque command TX during test mode.
  uint32_t user_tx_period_us = 1000;  // Command TX period in microseconds (1000 us = 1 kHz).

  SerialStats serial1_stats;                               // Telemetry serial performance stats

  LoopTimingStats timing;                                  // Loop timing / jitter stats
  uint32_t last_health_ms;                                 // Timestamp of last health update (ms)

  RCInputRaw rc_raw[RC_INPUT_MAX_PINS];                    // Raw RC inputs
  RCChannel  rc[RC_INPUT_MAX_PINS];                        // Normalized RC channels
  uint8_t    rc_count;                                     // Number of RC channels active
};

// ---------------- Globals / Prototypes ----------------
extern volatile uint32_t g_control_pending_ticks; // Pending control ISR ticks to service
extern volatile uint32_t g_control_now_us;// Timestamp of control loop trigger (µs)

void controlLoop_isr(void); // ISR triggered at CONTROL_PERIOD_US

void controlLoop(Supervisor_typedef *sup, // Core control loop logic
                 FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can1,
                 FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> &can2);
void test_can_transmit_mode(Supervisor_typedef *sup,
                            FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can1,
                            FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> &can2);

void init_supervisor(Supervisor_typedef *sup,
                     uint16_t esc_count,
                     const char *esc_names[],
                     const uint16_t node_ids[],
                     const uint8_t rc_pins[],
                     uint16_t rc_count);

void updateSupervisorRC(Supervisor_typedef *sup);   // Update RC input channels
void resetLoopTimingStats(Supervisor_typedef *sup); // Reset loop timing stats
void resetTelemetryStats(Supervisor_typedef *sup);  // Reset telemetry stats
float angle_diff(float target, float actual);


#endif
