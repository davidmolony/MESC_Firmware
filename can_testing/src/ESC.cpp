#include "ESC.h"

ESC* esc_lookup[ESC_LOOKUP_SIZE] = { nullptr };

ESC::ESC(const char* n, uint8_t id) {
    config.name = n;
    config.node_id = id;
}

void ESC::init() {
    state   = State{};
    command = Command{};
    status  = Status{};
}

void ESC::updateState(const State& s) {
    state = s;
    status.last_update_us = micros();
    status.alive = true;
}

void ESC::setCommand(float amps, int8_t dir) {
    command.amps_req = amps;
    command.direction = dir;
}

void ESC::setError(ESCError err) {
    status.errors = status.errors | err;
}

void ESC::clearError(ESCError err) {
    status.errors = static_cast<ESCError>(
        static_cast<uint32_t>(status.errors) & ~static_cast<uint32_t>(err)
    );
}

bool ESC::hasError(ESCError err) const {
    return (status.errors & err);
}
