#include "pushbutton.h"

PushButton::PushButton(uint8_t pin, bool use_pullup, uint32_t debounce_us)
    : pin(pin),
      invert(use_pullup),
      debounce_us(debounce_us ? debounce_us : 50000u),
      changed(false)
{
    if (use_pullup) pinMode(pin, INPUT_PULLUP);
    else            pinMode(pin, INPUT);

    uint32_t now = micros();
    PBState s = samplePressed();
    state = s;
    last_sampled = s;
    t_change_us = now;
}

PBState PushButton::samplePressed() const {
    int level = digitalReadFast(pin);
    bool pressed = invert ? (level == LOW) : (level == HIGH);
    return pressed ? PB_PRESSED : PB_RELEASED;
}

void PushButton::update(uint32_t now_us) {
    PBState inst = samplePressed();

    if (inst != last_sampled) {
        last_sampled = inst;
        t_change_us = now_us;
        return;
    }

    if (inst != state) {
        uint32_t dt = (uint32_t)(now_us - t_change_us);
        if (dt >= debounce_us) {
            state = inst;
            changed = true;
        }
        if (state == PB_PRESSED) {
            armed = true;   // mark as armed on press
        }
    }
}

PBState PushButton::readRaw() const {
    return samplePressed();
}
