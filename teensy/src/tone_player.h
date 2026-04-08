#ifndef TONE_PLAYER_H
#define TONE_PLAYER_H

#include <Arduino.h>

static constexpr uint32_t PB_BEEP_HZ = 2000;
static constexpr uint32_t PB_BEEP_MS = 150;
static constexpr uint32_t PB_GAP_MS  = 100;

typedef enum {
  TONE_IDLE = 0,     // nothing happening
  TONE_PLAYING,      // toggling pin at frequency
  TONE_SILENCE       // post-tone quiet period
} ToneState;

typedef struct {
  uint8_t   pin;
  volatile ToneState state;
  uint32_t  play_end_us;      // when the tone should stop
  uint32_t  silence_end_us;   // when the silence ends
  uint32_t  freq_hz;          // current tone frequency
} TonePlayer;

// Initialize the player for a specific pin (one speaker supported)
void tone_init(TonePlayer* tp, uint8_t pin);

// Start a tone: freq_hz for dur_ms, then silence_ms of silence.
// freq_hz == 0 starts only the silence period (no tone).
void tone_start(TonePlayer* tp, uint32_t freq_hz, uint32_t dur_ms, uint32_t silence_ms);

// Stop immediately (no post-silence)
void tone_stop(TonePlayer* tp);

// Call this frequently (e.g., once per loop) to advance the state machine.
void tone_update(TonePlayer* tp, uint32_t now_us);

// Polling helpers
static inline bool tone_is_playing(const TonePlayer* tp)  { return tp->state == TONE_PLAYING; }
static inline bool tone_is_silence(const TonePlayer* tp)  { return tp->state == TONE_SILENCE; }
static inline bool tone_is_idle(const TonePlayer* tp)     { return tp->state == TONE_IDLE;    }

#endif // TONE_PLAYER_H
