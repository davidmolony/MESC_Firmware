#ifndef PUSHBUTTON_H
#define PUSHBUTTON_H

#include <Arduino.h>

enum PBState {
    PB_RELEASED = 0,
    PB_PRESSED  = 1
};

class PushButton {
public:
    PushButton(uint8_t pin, bool use_pullup, uint32_t debounce_us = 50000);

    void update(uint32_t now_us);
    PBState readRaw() const;

    // Accessors
    PBState getState() const { return state; }
    bool hasChanged() const { return changed; }
    void clearChanged() { changed = false; }
    bool isArmed() const { return armed; }
    void clearArmed() { armed = false; }

private:
    PBState samplePressed() const;

    uint8_t pin;
    bool invert;
    uint32_t debounce_us;
    PBState state;
    PBState last_sampled;
    uint32_t t_change_us;
    bool changed;
    bool armed = false; 
};

#endif // PUSHBUTTON_H
