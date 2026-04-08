#include "LED.h"

void led_init(LEDCtrl* l, uint8_t pin, LEDState initial) {
  l->pin = pin;
  l->state = initial;
  l->next_toggle_us = micros();
  l->level = (initial == LED_ON_CONTINUOUS);  // ON starts high, others low
  pinMode(pin, OUTPUT);
  digitalWriteFast(pin, l->level ? HIGH : LOW);
}

void led_update(LEDCtrl* l, uint32_t now_us) {
  switch (l->state) {
    case LED_OFF:
      if (l->level) { l->level = false; digitalWriteFast(l->pin, LOW); }
      break;

    case LED_ON_CONTINUOUS:
      if (!l->level) { l->level = true; digitalWriteFast(l->pin, HIGH); }
      break;

    case LED_BLINK_SLOW:
    case LED_BLINK_FAST: {
      const uint32_t half = (l->state == LED_BLINK_FAST) ? LED_FAST_HALF_US : LED_SLOW_HALF_US;
      if ((int32_t)(now_us - l->next_toggle_us) >= 0) {
        l->level = !l->level;
        digitalWriteFast(l->pin, l->level ? HIGH : LOW);
        l->next_toggle_us += half;          // keep cadence (no drift)
      }
    } break;

    case LED_PULSE: {
      const uint32_t t = now_us % LED_PULSE_PERIOD_US;
      const uint32_t half = LED_PULSE_PERIOD_US / 2u;
      uint32_t duty = (t < half)
        ? (t * 255u) / half
        : ((LED_PULSE_PERIOD_US - t) * 255u) / half;
      analogWrite(l->pin, (int)duty);
      l->level = (duty >= 128u);
    } break;
  }
}
