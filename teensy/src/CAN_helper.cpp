#include "CAN_helper.h"
#include <math.h>
#include <string.h>

#define INVERT_ESC_ENCODER  0

static volatile uint32_t g_last_posvel_rx_us = 0;
static PosvelRxStats g_posvel_stats[ESC_LOOKUP_SIZE];
static PosvelSeqStats g_posvel_seq_stats[ESC_LOOKUP_SIZE];
static PosvelRxStats g_bus_posvel_stats[3]; // index 1=CAN1, 2=CAN2
static constexpr uint32_t GAP_HIST_BIN_US = 100u;
static constexpr uint32_t GAP_HIST_MAX_US = 20000u;
static constexpr uint32_t GAP_HIST_BINS = (GAP_HIST_MAX_US / GAP_HIST_BIN_US) + 1u;
struct GapHistogram {
    uint32_t bins[GAP_HIST_BINS];
};
static GapHistogram g_posvel_hist[ESC_LOOKUP_SIZE];
static GapHistogram g_bus_posvel_hist[3]; // index 1=CAN1, 2=CAN2
static volatile uint32_t g_can1_rx_reads = 0;
static volatile uint32_t g_can2_rx_reads = 0;
static volatile uint32_t g_rx_overflow = 0;
static constexpr uint32_t POSVEL_EXPECTED_PERIOD_US = 2000u;

struct SpikeRing {
    uint32_t ts[POSVEL_GAP_SPIKE_RING_SIZE];
    uint16_t head = 0;
    uint16_t size = 0;
    uint32_t total = 0;
};
static SpikeRing g_posvel_gap_spikes[ESC_LOOKUP_SIZE];

static inline void reset_gap_hist(GapHistogram &hist) {
    memset(&hist, 0, sizeof(hist));
}

static inline void reset_spike_ring(SpikeRing &ring) {
    ring.head = 0u;
    ring.size = 0u;
    ring.total = 0u;
    memset(ring.ts, 0, sizeof(ring.ts));
}

static inline void spike_ring_add(SpikeRing &ring, uint32_t ts_us) {
    if (ring.size < POSVEL_GAP_SPIKE_RING_SIZE) {
        const uint16_t idx = (uint16_t)((ring.head + ring.size) % POSVEL_GAP_SPIKE_RING_SIZE);
        ring.ts[idx] = ts_us;
        ring.size++;
    } else {
        ring.ts[ring.head] = ts_us;
        ring.head = (uint16_t)((ring.head + 1u) % POSVEL_GAP_SPIKE_RING_SIZE);
    }
    if (ring.total < UINT32_MAX) ring.total++;
}

static inline uint32_t gap_hist_bin_for_gap(uint32_t gap_us) {
    uint32_t idx = gap_us / GAP_HIST_BIN_US;
    if (idx >= GAP_HIST_BINS) idx = GAP_HIST_BINS - 1u;
    return idx;
}

static inline void gap_hist_add(GapHistogram &hist, uint32_t gap_us) {
    hist.bins[gap_hist_bin_for_gap(gap_us)]++;
}

static uint32_t gap_hist_percentile_us(const GapHistogram &hist, uint32_t sample_count, float q) {
    if (sample_count == 0u) return 0u;
    if (q < 0.0f) q = 0.0f;
    if (q > 1.0f) q = 1.0f;

    uint32_t target = (uint32_t)(q * (float)sample_count + 0.5f);
    if (target == 0u) target = 1u;
    if (target > sample_count) target = sample_count;

    uint32_t acc = 0u;
    for (uint32_t i = 0u; i < GAP_HIST_BINS; ++i) {
        acc += hist.bins[i];
        if (acc >= target) {
            if (i == GAP_HIST_BINS - 1u) {
                return GAP_HIST_MAX_US;
            }
            return (i * GAP_HIST_BIN_US) + (GAP_HIST_BIN_US / 2u);
        }
    }
    return GAP_HIST_MAX_US;
}

static inline void reset_posvel_stat(PosvelRxStats &st) {
    st.count = 0;
    st.last_rx_us = 0;
    st.min_gap_us = UINT32_MAX;
    st.p50_gap_us = 0;
    st.p95_gap_us = 0;
    st.p99_gap_us = 0;
    st.max_gap_us = 0;
    st.sum_gap_us = 0;
    st.gap_count = 0;
    st.est_missed = 0;
}

static inline void reset_posvel_seq_stat(PosvelSeqStats &st) {
    st.valid_count = 0u;
    st.last_seq = 0u;
    st.has_last = 0u;
    st.missed_total = 0u;
    st.duplicate_total = 0u;
    st.out_of_order_total = 0u;
    st.burst_miss_max = 0u;
}

static inline void update_posvel_stat(PosvelRxStats &st,
                                      GapHistogram &hist,
                                      uint32_t now_us,
                                      SpikeRing *spike_ring) {
    if (st.last_rx_us != 0u) {
      const uint32_t gap_us = (uint32_t)(now_us - st.last_rx_us);
      if (gap_us < st.min_gap_us) st.min_gap_us = gap_us;
      if (gap_us > st.max_gap_us) st.max_gap_us = gap_us;
      st.sum_gap_us += gap_us;
      st.gap_count++;
      gap_hist_add(hist, gap_us);
      if (spike_ring != nullptr && gap_us >= POSVEL_GAP_SPIKE_US) {
        spike_ring_add(*spike_ring, now_us);
      }

      // Approximate missed frames based on expected 500 Hz cadence.
      if (gap_us > (POSVEL_EXPECTED_PERIOD_US + POSVEL_EXPECTED_PERIOD_US / 2u)) {
        const uint32_t periods = (gap_us + POSVEL_EXPECTED_PERIOD_US / 2u) / POSVEL_EXPECTED_PERIOD_US;
        if (periods > 1u) st.est_missed += (periods - 1u);
      }
    }
    st.last_rx_us = now_us;
    st.count++;
}

static inline bool decode_posvel_seq(float seq_f, uint32_t &out_seq) {
    if (!isfinite(seq_f) || seq_f < 0.0f) return false;
    const uint32_t seq_i = (uint32_t)(seq_f + 0.5f);
    if (fabsf(seq_f - (float)seq_i) > 0.01f) return false;
    out_seq = seq_i;
    return true;
}

static inline void update_posvel_seq_stat(PosvelSeqStats &st, uint32_t seq) {
    st.valid_count++;
    if (st.has_last == 0u) {
      st.last_seq = seq;
      st.has_last = 1u;
      return;
    }

    if (seq == st.last_seq) {
      st.duplicate_total++;
      return;
    }

    if (seq > st.last_seq) {
      const uint32_t delta = seq - st.last_seq;
      if (delta > 1u) {
        const uint32_t missed = delta - 1u;
        st.missed_total += missed;
        if (missed > st.burst_miss_max) st.burst_miss_max = missed;
      }
      st.last_seq = seq;
      return;
    }

    st.out_of_order_total++;
}

bool canBufferPush(CANBuffer &cb, const CAN_message_t &msg, uint8_t rx_bus) {
    int next = (cb.head + 1) % CAN_BUF_SIZE;
    if (next == cb.tail) {
      cb.overflow_count++; // nothing reads this at this point
      return false;
    }

    cb.buf[cb.head] = msg;
    cb.bus[cb.head] = rx_bus;
    cb.head = next;
    cb.link_ok = true;
    return true;
}

bool canBufferPop(CANBuffer &cb, CAN_message_t &msg, uint8_t *rx_bus) {
    if (cb.head == cb.tail) return false;
    msg = cb.buf[cb.tail];
    if (rx_bus != nullptr) {
      *rx_bus = cb.bus[cb.tail];
    }
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
    if (out.gap_count > 0u) {
      out.p50_gap_us = gap_hist_percentile_us(g_posvel_hist[node_id], out.gap_count, 0.50f);
      out.p95_gap_us = gap_hist_percentile_us(g_posvel_hist[node_id], out.gap_count, 0.95f);
      out.p99_gap_us = gap_hist_percentile_us(g_posvel_hist[node_id], out.gap_count, 0.99f);
    }
    return true;
}

bool canGetPosvelSeqStats(uint8_t node_id, PosvelSeqStats &out) {
    if (node_id >= ESC_LOOKUP_SIZE) return false;
    out = g_posvel_seq_stats[node_id];
    return true;
}

bool canGetBusPosvelRxStats(uint8_t bus, PosvelRxStats &out) {
    if (bus > 2u || bus == 0u) return false;
    out = g_bus_posvel_stats[bus];
    if (out.gap_count > 0u) {
      out.p50_gap_us = gap_hist_percentile_us(g_bus_posvel_hist[bus], out.gap_count, 0.50f);
      out.p95_gap_us = gap_hist_percentile_us(g_bus_posvel_hist[bus], out.gap_count, 0.95f);
      out.p99_gap_us = gap_hist_percentile_us(g_bus_posvel_hist[bus], out.gap_count, 0.99f);
    }
    return true;
}

void canResetPosvelStats() {
    for (uint8_t i = 0; i < ESC_LOOKUP_SIZE; ++i) {
      reset_posvel_stat(g_posvel_stats[i]);
      reset_posvel_seq_stat(g_posvel_seq_stats[i]);
      reset_gap_hist(g_posvel_hist[i]);
      reset_spike_ring(g_posvel_gap_spikes[i]);
    }
    reset_posvel_stat(g_bus_posvel_stats[1]);
    reset_posvel_stat(g_bus_posvel_stats[2]);
    reset_gap_hist(g_bus_posvel_hist[1]);
    reset_gap_hist(g_bus_posvel_hist[2]);
    g_last_posvel_rx_us = 0;
}

uint32_t canGetPosvelGapSpikeThresholdUs() {
    return POSVEL_GAP_SPIKE_US;
}

uint32_t canGetPosvelGapSpikeCount(uint8_t node_id) {
    if (node_id >= ESC_LOOKUP_SIZE) return 0u;
    return g_posvel_gap_spikes[node_id].total;
}

uint16_t canCopyPosvelGapSpikeTimes(uint8_t node_id, uint32_t *out, uint16_t max_out) {
    if (node_id >= ESC_LOOKUP_SIZE || out == nullptr || max_out == 0u) return 0u;
    const SpikeRing &ring = g_posvel_gap_spikes[node_id];
    uint16_t n = ring.size;
    if (n > max_out) n = max_out;
    const uint16_t start = (uint16_t)((ring.head + ring.size - n) % POSVEL_GAP_SPIKE_RING_SIZE);
    for (uint16_t i = 0u; i < n; ++i) {
      out[i] = ring.ts[(uint16_t)((start + i) % POSVEL_GAP_SPIKE_RING_SIZE)];
    }
    return n;
}

void canNoteBusRead(uint8_t bus) {
    if (bus == 1u) {
      g_can1_rx_reads++;
    } else if (bus == 2u) {
      g_can2_rx_reads++;
    }
}

void canNoteRxOverflow() {
    g_rx_overflow++;
}

CanRuntimeStats canGetRuntimeStats() {
    CanRuntimeStats out{};
    out.can1_rx_reads = g_can1_rx_reads;
    out.can2_rx_reads = g_can2_rx_reads;
    out.rx_overflow = g_rx_overflow;
    return out;
}

void canResetRuntimeStats() {
    g_can1_rx_reads = 0;
    g_can2_rx_reads = 0;
    g_rx_overflow = 0;
}

CAN_message_t g_last_can_msg = {};
void handleCANMessage(const CAN_message_t &msg, uint8_t rx_bus) {
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
            uint32_t seq = 0u;
            const uint32_t now_us = micros();
            PosvelRxStats &st = g_posvel_stats[sender_id];
            update_posvel_stat(st, g_posvel_hist[sender_id], now_us, &g_posvel_gap_spikes[sender_id]);
            if (decode_posvel_seq(pos, seq)) {
              update_posvel_seq_stat(g_posvel_seq_stats[sender_id], seq);
            }
            if (rx_bus == 1u || rx_bus == 2u) {
              update_posvel_stat(g_bus_posvel_stats[rx_bus], g_bus_posvel_hist[rx_bus], now_us, nullptr);
            }
            g_last_posvel_rx_us = now_us;
#if INVERT_ESC_ENCODER
	    esc->state.pos_rad   = TWO_PI - pos;  // mirror around 2π
	    esc->state.vel_rad_s = -vel;          // flip velocity sign
#else
	    esc->state.pos_rad   = pos;
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
