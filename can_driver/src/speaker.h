#ifndef SPEAKER_H
#define SPEAKER_H

#include <Arduino.h>
#include <stddef.h>  // size_t

void speaker_start_tone(uint16_t freq_hz);  // starts continuous tone at freq
void speaker_stop_tone(void);               // stops any tone immediately

typedef enum {
  SPEAKER_SILENT = 0,
  SPEAKER_PLAY_ONCE,
  SPEAKER_PLAY_CONTINUOUS,
  SPEAKER_TIE_TO_BALANCE
} SpeakerMode;

typedef struct {
  uint8_t      pin;

  // Bound melody (arrays owned by caller)
  const int   *melody;      // Hz per note (use NOTE_* from tones.h)
  const int   *dur_ms;      // duration per note, in milliseconds
  size_t       length;

  // Playback state
  SpeakerMode  mode;
  size_t       idx;         // current note index
  uint32_t     note_end_us; // timestamp when current note should end
  bool         playing;     // true when tone is sounding

} Speaker;

void speaker_init(Speaker *spk, uint8_t pin);

// Bind a melody (arrays must outlive the speaker)
void speaker_bind_melody(Speaker *spk, const int *melody_hz, const int *dur_ms, size_t length);

// Change playback mode
void speaker_set_mode(Speaker *spk, SpeakerMode mode);

// Call often (e.g., once per loop). balance_active is used only for TIE_TO_BALANCE.
void speaker_update(Speaker *spk, uint32_t now_us, bool balance_active);

#endif // SPEAKER_H
