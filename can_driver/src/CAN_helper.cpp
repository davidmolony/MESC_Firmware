#include "CAN_helper.h"

#include <math.h>

#define INVERT_ESC_ENCODER  0

static volatile uint32_t g_last_posvel_rx_us = 0;
static PosvelRxStats g_posvel_stats[ESC_LOOKUP_SIZE];
static constexpr uint32_t POSVEL_EXPECTED_PERIOD_US = 2000u;
static bool g_posvel_counter_mode = false;

bool canBufferPush(CANBuffer &cb, const CAN_message_t &msg) {
    int next = (cb.head + 1) % CAN_BUF_SIZE;
    if (next == cb.tail) {
      cb.overflow_count++; // nothing reads this at this point
      return false;
    }

    cb.buf[cb.head] = msg;
    cb.head = next;
    cb.link_ok = true;
    return true;
}

bool canBufferPop(CANBuffer &cb, CAN_message_t &msg) {
    if (cb.head == cb.tail) return false;
    msg = cb.buf[cb.tail];
    cb.tail = (cb.tail + 1) % CAN_BUF_SIZE;
    return true;
}

// Extract message type (bits 28..16)
uint16_t extractMsgType(uint32_t can_id) {
    return (can_id >> 16) & 0x1FFF;  // 13-bit field
}

// Extract receiver node (bits 15..8)
uint8_t extractReceiver(uint32_t can_id) {
    return (can_id >> 8) & 0xFF;
}

// Extract sender node (bits 7..0)
uint8_t extractSender(uint32_t can_id) {
    return can_id & 0xFF;
}

float extractFloat(const uint8_t *buf) {
    float val;
    memcpy(&val, buf, sizeof(float));
    return val;
}

// Build full 29-bit CAN extended ID
uint32_t canMakeExtId(uint16_t msg_id, uint8_t sender, uint8_t receiver) {
    return ((uint32_t)msg_id << 16) |
           ((uint32_t)receiver << 8) |
           sender;
}

// Encode float into buffer
void canPackFloat(float val, uint8_t *buf) {
  memcpy(buf, &val, sizeof(float));
}

uint32_t canGetLastPosVelRxUs() {
    return g_last_posvel_rx_us;
}

bool canGetPosvelRxStats(uint8_t node_id, PosvelRxStats &out) {
    if (node_id >= ESC_LOOKUP_SIZE) return false;
    out = g_posvel_stats[node_id];
    return true;
}

void canResetPosvelRxStats() {
    for (uint16_t i = 0; i < ESC_LOOKUP_SIZE; ++i) {
        g_posvel_stats[i] = PosvelRxStats{};
    }
    g_last_posvel_rx_us = 0;
}

void canSetPosvelCounterMode(bool enabled) {
    g_posvel_counter_mode = enabled;
}

bool canGetPosvelCounterMode() {
    return g_posvel_counter_mode;
}

CAN_message_t g_last_can_msg = {};
void handleCANMessage(const CAN_message_t &msg) {
    g_last_can_msg = msg;
    uint16_t msg_type = extractMsgType(msg.id);
    uint8_t sender_id = extractSender(msg.id);   // ESC ID
    // uint8_t receiver  = extractReceiver(msg.id); // so far we never test this


    // Serial.printf("[CAN RX] raw_id=0x%08X msg_type=0x%X sender=%u receiver=%u\r\n",
    //    msg.id, msg_type, sender_id, receiver);

    if (sender_id >= ESC_LOOKUP_SIZE || !esc_lookup[sender_id]) {
      // Serial.printf("[CAN RX] sender_id %u not mapped\r\n", sender_id);
      return;
    }
    ESC* esc = esc_lookup[sender_id];

    switch (msg_type) {
        case CAN_ID_POSVEL: {
	    float pos, vel;
	    memcpy(&pos, &msg.buf[0], sizeof(float));
	    memcpy(&vel, &msg.buf[4], sizeof(float));
            const uint32_t now_us = micros();
            PosvelRxStats &st = g_posvel_stats[sender_id];
            if (st.last_rx_us != 0u) {
              const uint32_t gap_us = (uint32_t)(now_us - st.last_rx_us);
              if (gap_us < st.min_gap_us) st.min_gap_us = gap_us;
              if (gap_us > st.max_gap_us) st.max_gap_us = gap_us;
              st.sum_gap_us += gap_us;
              st.gap_count++;

              // Approximate missed frames based on expected 500 Hz cadence.
              if (gap_us > (POSVEL_EXPECTED_PERIOD_US + POSVEL_EXPECTED_PERIOD_US / 2u)) {
                const uint32_t periods = (gap_us + POSVEL_EXPECTED_PERIOD_US / 2u) / POSVEL_EXPECTED_PERIOD_US;
                if (periods > 1u) st.est_missed += (periods - 1u);
              }
            }
            st.last_rx_us = now_us;
            st.count++;
            g_last_posvel_rx_us = now_us;

                        if (g_posvel_counter_mode) {
                            const uint32_t ctr = (uint32_t)lroundf(pos);
                            if (st.counter_seen) {
                                if (ctr == st.last_counter) {
                                    st.counter_duplicates++;
                                } else {
                                    const uint32_t delta = ctr - st.last_counter;
                                    if (delta > 1u) {
                                        st.counter_jumps++;
                                        st.counter_jump_total += (delta - 1u);
                                    }
                                }
                            }
                            st.last_counter = ctr;
                            st.counter_seen = true;
                        }
#if INVERT_ESC_ENCODER
	    esc->state.pos_rad   = TWO_PI - pos;  // mirror around 2π
	    esc->state.vel_rad_s = -vel;          // flip velocity sign
#else
	    if (!g_posvel_counter_mode) {
	      esc->state.pos_rad = pos;
	    }
	    esc->state.vel_rad_s = vel;
#endif
	    esc->state.alive     = true;
            esc->status.last_update_us = now_us;
            esc->status.alive = true;
	    // Serial.printf("{\"cmd\": \"PRINT\", \"note\": \"Torque response received CAN\"}\n");

	    // Serial.printf("[CAN RX] POSVEL sender=%u pos=%.3f rad vel=%.3f rad/s\r\n", sender_id, pos, vel);

            break;
        }
        case CAN_ID_TEMPS: {
            float mos, mot;
            memcpy(&mos, &msg.buf[0], sizeof(float));
            memcpy(&mot, &msg.buf[4], sizeof(float));
            esc->state.temp_mos = mos;
            esc->state.temp_mot = mot;
            esc->state.alive    = true;   // ✅ also valid here
            break;
        }
        default:
            break;
    }
}
