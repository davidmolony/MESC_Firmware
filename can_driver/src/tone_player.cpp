#include "tone_player.h"
#include <IntervalTimer.h>

// Single speaker: one IntervalTimer + one pin.
static IntervalTimer s_tpTimer;
static uint8_t       s_tpPin   = 255;
static volatile bool s_level   = false;

static void tp_isr_toggle() {
  s_level = !s_level;
  digitalWriteFast(s_tpPin, s_level ? HIGH : LOW);
}

static void tp_start_osc(uint32_t freq_hz) {
  if (freq_hz == 0 || s_tpPin == 255) {
    s_tpTimer.end();
    digitalWriteFast(s_tpPin, LOW);
    s_level = false;
    return;
  }
  uint32_t half_us = 500000UL / (freq_hz ? freq_hz : 1);
  if (half_us == 0) half_us = 1;

  s_level = false;
  digitalWriteFast(s_tpPin, LOW);

  // (re)start the PIT channel and set a solid priority
  if (s_tpTimer.begin(tp_isr_toggle, half_us)) {
    s_tpTimer.priority(64);   // 0 = highest, 255 = lowest; 64 is safely above USB
  }
}

static void tp_stop_osc() {
  s_tpTimer.end();
  digitalWriteFast(s_tpPin, LOW);
  s_level = false;
}

void tone_init(TonePlayer* tp, uint8_t pin) {
  tp->pin = pin;
  tp->state = TONE_IDLE;
  tp->play_end_us = 0;
  tp->silence_end_us = 0;
  tp->freq_hz = 0;

  s_tpPin = pin;
  pinMode(pin, OUTPUT);
  digitalWriteFast(pin, LOW);
}

void tone_start(TonePlayer* tp, uint32_t freq_hz, uint32_t dur_ms, uint32_t silence_ms) {
  tp->freq_hz = freq_hz;

  const uint32_t now = micros();
  if (freq_hz > 0 && dur_ms > 0) {
    // Start the tone immediately
    tp->play_end_us = now + (dur_ms * 1000UL);
    tp_start_osc(freq_hz);
    tp->state = TONE_PLAYING;
  } else {
    // No tone: go straight to silence if requested
    tp_stop_osc();
    if (silence_ms > 0) {
      tp->silence_end_us = now + (silence_ms * 1000UL);
      tp->state = TONE_SILENCE;
    } else {
      tp->state = TONE_IDLE;
    }
  }

  // Always program the post-tone silence (even if 0)
  tp->silence_end_us = now + (dur_ms * 1000UL) + (silence_ms * 1000UL);
}

void tone_stop(TonePlayer* tp) {
  tp_stop_osc();
  tp->state = TONE_IDLE;
  tp->play_end_us = 0;
  tp->silence_end_us = 0;
  tp->freq_hz = 0;
}

void tone_update(TonePlayer* tp, uint32_t now_us) {
  switch (tp->state) {
    case TONE_IDLE:
      // nothing to do
      break;

    case TONE_PLAYING:
      if ((int32_t)(now_us - tp->play_end_us) >= 0) {
        // End of tone -> start/post silence (or go idle if silence already elapsed)
        tp_stop_osc();
        if ((int32_t)(now_us - tp->silence_end_us) >= 0) {
          tp->state = TONE_IDLE;
        } else {
          tp->state = TONE_SILENCE;
        }
      }
      break;

    case TONE_SILENCE:
      if ((int32_t)(now_us - tp->silence_end_us) >= 0) {
        tp->state = TONE_IDLE;
      }
      break;
  }
}
