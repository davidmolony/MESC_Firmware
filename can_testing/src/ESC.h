#ifndef ESC_H
#define ESC_H

#include <Arduino.h>
#include <stdint.h>

// max node_IDs supported
//  notice this number is larger than TELEMETRY_ESC_MAX
//  which is the number ESCs that are sent to system telemtry

#define ESC_LOOKUP_SIZE 16  

extern class ESC* esc_lookup[ESC_LOOKUP_SIZE];

// ---------------- ESC Error Flags ----------------
enum class ESCError : uint32_t {
  NONE        = 0x00,
  OVERHEAT    = 0x01,
  OVERCURRENT = 0x02,
  CAN_TIMEOUT = 0x04,
  RESPONSE    = 0x08
};

inline ESCError operator|(ESCError a, ESCError b) {
  return static_cast<ESCError>(
			       static_cast<uint32_t>(a) | static_cast<uint32_t>(b)
			       );
}
inline bool operator&(ESCError a, ESCError b) {
  return (static_cast<uint32_t>(a) & static_cast<uint32_t>(b)) != 0;
}

// ---------------- ESC Class ----------------
class ESC {
public:
  // ---- Config (static parameters) ----
  struct Config {
    const char* name = nullptr;
    uint8_t node_id = 0;
    uint8_t pole_pairs = 0;
    float encoder_offset = 0.0f;
    float volts_max = 0.0f;
    float volts_min = 0.0f;
    float amps_max = 0.0f;
    int8_t direction = 1;
  };

  // ---- State (dynamic telemetry) ----
  struct State {
    float pos_rad = 0.0f;
    float vel_rad_s = 0.0f;
    float bus_voltage = 0.0f;
    float bus_current = 0.0f;
    float motor_current = 0.0f;
    float motor_voltage = 0.0f;
    float foc_angle = 0.0f;
    float temp_mos = 0.0f;
    float temp_mot = 0.0f;
    uint32_t error_code = 0;
    bool alive;
  };

  // ---- Command (desired outputs) ----
  struct Command {
    float amps_req = 0.0f;
    int8_t direction = 1;
  };

  // ---- Status (meta info) ----
  struct Status {
    ESCError errors = ESCError::NONE;
    bool alive = false;
    uint32_t last_update_us = 0;
  };

  // ---- Methods ----
  ESC(const char* name = nullptr, uint8_t node_id = 0);
  void init();
  void updateState(const State& s);
  void setCommand(float amps, int8_t dir = 1);
  void setError(ESCError err);
  void clearError(ESCError err);
  bool hasError(ESCError err) const;

  // ---- Public fields ----
  Config config;
  State state;
  Command command;
  Status status;
};

#endif // ESC_H
