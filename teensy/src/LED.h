#ifndef LED_H
#define LED_H

#include <Arduino.h>

// ===== LED states & timing =====
typedef enum {
  LED_ON_CONTINUOUS = 0,
  LED_OFF,
  LED_BLINK_SLOW,
  LED_BLINK_FAST,
  LED_PULSE
} LEDState;

#ifndef LED_SLOW_HALF_US
#define LED_SLOW_HALF_US 500000u   // 0.5 s on, 0.5 s off
#endif
#ifndef LED_FAST_HALF_US
#define LED_FAST_HALF_US 100000u   // 0.1 s on, 0.1 s off
#endif
#ifndef LED_PULSE_PERIOD_US
#define LED_PULSE_PERIOD_US 1200000u // 1.2 s full breathe cycle
#endif

typedef struct {
  uint8_t  pin;
  volatile LEDState state;
  uint32_t next_toggle_us;
  bool     level;         // current output level
} LEDCtrl;

// API
void led_init(LEDCtrl* l, uint8_t pin, LEDState initial);
static inline void led_set_state(LEDCtrl* l, LEDState s) { l->state = s; }
void led_update(LEDCtrl* l, uint32_t now_us);

#endif // LED_H
