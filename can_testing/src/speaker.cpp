#include "speaker.h"
#include <IntervalTimer.h>

static const uint8_t PIN_INVALID = 0xFF;
static IntervalTimer s_toneTimer;
static uint8_t       s_spkPin = PIN_INVALID;
static volatile bool s_level  = false;

static void isr_toggle() {
  s_level = !s_level;
  digitalWriteFast(s_spkPin, s_level ? HIGH : LOW);
}

static void start_tone(uint16_t freq_hz) {
  if (freq_hz == 0 || s_spkPin == PIN_INVALID) {
    s_toneTimer.end();
    if (s_spkPin != PIN_INVALID) digitalWriteFast(s_spkPin, LOW);
    s_level = false;
    return;
  }

  uint32_t half_us = 500000UL / (uint32_t)freq_hz;
  if (half_us == 0) half_us = 1;

  s_level = false;
  digitalWriteFast(s_spkPin, LOW);

  bool ok = s_toneTimer.begin(isr_toggle, half_us);
  if (!ok) {
    if (s_spkPin != PIN_INVALID) {
      digitalWriteFast(s_spkPin, HIGH); delay(10);
      digitalWriteFast(s_spkPin, LOW);
    }
  } else {
    s_toneTimer.priority(128);
  }
}

static void stop_tone() {
  s_toneTimer.end();
  if (s_spkPin != PIN_INVALID) digitalWriteFast(s_spkPin, LOW);
  s_level = false;
}

void speaker_init(Speaker *spk, uint8_t pin) {
  spk->pin = pin;
  s_spkPin = pin;
  pinMode(pin, OUTPUT);
  digitalWriteFast(pin, LOW);
}


void speaker_bind_melody(Speaker *spk, const int *melody_hz, const int *dur_ms, size_t length) {
  spk->melody = melody_hz;
  spk->dur_ms = dur_ms;
  spk->length = length;
  spk->idx = 0;
  spk->note_end_us = 0;
  spk->playing = false;
}

void speaker_set_mode(Speaker *spk, SpeakerMode mode) {
  spk->mode = mode;
  // Reset sequence if starting a play mode
  if (mode == SPEAKER_PLAY_ONCE || mode == SPEAKER_PLAY_CONTINUOUS || mode == SPEAKER_TIE_TO_BALANCE) {
    spk->idx = 0;
    spk->note_end_us = 0;
    spk->playing = false;
  }
  if (mode == SPEAKER_SILENT) {
    stop_tone();
    spk->playing = false;
  }
}

static inline void advance_or_stop(Speaker *spk, uint32_t now_us, bool loop_after) {
  // Advance to next note or stop/end/loop
  spk->idx++;
  if (spk->idx >= spk->length || spk->melody == nullptr || spk->dur_ms == nullptr) {
    // Sequence completed
    if (loop_after) {
      spk->idx = 0;
    } else {
      stop_tone();
      spk->playing = false;
      spk->idx = 0;
      spk->note_end_us = 0;
      spk->mode = (spk->mode == SPEAKER_PLAY_ONCE) ? SPEAKER_SILENT : spk->mode;
      return;
    }
  }
  // Start next note
  const int freq = spk->melody[spk->idx];
  const int dur  = spk->dur_ms[spk->idx];
  start_tone((uint16_t)((freq > 0) ? freq : 0)); // rest if freq<=0
  spk->playing = (freq > 0);
  spk->note_end_us = now_us + (uint32_t)dur * 1000UL;
}

void speaker_update(Speaker *spk, uint32_t now_us, bool balance_active) {
  const bool have_mel = (spk->melody && spk->dur_ms && spk->length > 0);

  switch (spk->mode) {
    case SPEAKER_SILENT:
      // nothing
      return;

    case SPEAKER_PLAY_CONTINUOUS:
    case SPEAKER_PLAY_ONCE: {
      if (!have_mel) {
        stop_tone();
        spk->playing = false;
        return;
      }

      const bool loop_after = (spk->mode == SPEAKER_PLAY_CONTINUOUS) ||
                              (spk->mode == SPEAKER_TIE_TO_BALANCE);

      if (!spk->playing) {
        // (Re)start sequence from current idx
        const int freq = spk->melody[spk->idx];
        const int dur  = spk->dur_ms[spk->idx];
        start_tone((uint16_t)((freq > 0) ? freq : 0));
        spk->playing = (freq > 0);
        spk->note_end_us = now_us + (uint32_t)dur * 1000UL;
      } else if ((int32_t)(now_us - spk->note_end_us) >= 0) {
        // Time for next note
        advance_or_stop(spk, now_us, loop_after);
      }
    } break;
  }
}

void speaker_start_tone(uint16_t freq_hz) { start_tone(freq_hz); }
void speaker_stop_tone(void)              { stop_tone(); }
