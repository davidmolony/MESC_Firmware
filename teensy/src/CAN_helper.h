#ifndef CAN_HELPER_H
#define CAN_HELPER_H

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "ESC.h"

#define CAN_BUF_SIZE 256
#define CAN_ID_POSVEL 0x2D0
#define CAN_ID_TEMPS  0x2D1
#define POSVEL_GAP_SPIKE_US 3000
#define POSVEL_GAP_SPIKE_RING_SIZE 128

struct CANBuffer {
    CAN_message_t buf[CAN_BUF_SIZE];
    uint8_t bus[CAN_BUF_SIZE];
    volatile int head = 0;
    volatile int tail = 0;
    volatile int overflow_count = 0;
    bool link_ok = false;
};

struct PosvelRxStats {
  uint32_t count = 0;
  uint32_t last_rx_us = 0;
  uint32_t min_gap_us = UINT32_MAX;
  uint32_t p50_gap_us = 0;
  uint32_t p95_gap_us = 0;
  uint32_t p99_gap_us = 0;
  uint32_t max_gap_us = 0;
  uint64_t sum_gap_us = 0;
  uint32_t gap_count = 0;
  uint32_t est_missed = 0;
};

struct PosvelSeqStats {
  uint32_t valid_count = 0;
  uint32_t last_seq = 0;
  uint8_t has_last = 0;
  uint32_t missed_total = 0;
  uint32_t duplicate_total = 0;
  uint32_t out_of_order_total = 0;
  uint32_t burst_miss_max = 0;
};

struct CanRuntimeStats {
  uint32_t can1_rx_reads = 0;
  uint32_t can2_rx_reads = 0;
  uint32_t rx_overflow = 0;
};

bool canBufferPush(CANBuffer &cb, const CAN_message_t &msg, uint8_t rx_bus);
bool canBufferPop(CANBuffer &cb, CAN_message_t &msg, uint8_t *rx_bus);

uint8_t extractNodeID(uint32_t can_id);
uint16_t extractMsgType(uint32_t can_id);
float extractFloat(const uint8_t *buf);

uint32_t canMakeExtId(uint16_t msg_id, uint8_t sender, uint8_t receiver);
void canPackFloat(float val, uint8_t *buf);
uint32_t canGetLastPosVelRxUs();
bool canGetPosvelRxStats(uint8_t node_id, PosvelRxStats &out);
bool canGetPosvelSeqStats(uint8_t node_id, PosvelSeqStats &out);
bool canGetBusPosvelRxStats(uint8_t bus, PosvelRxStats &out);
void canResetPosvelStats();
uint32_t canGetPosvelGapSpikeThresholdUs();
uint32_t canGetPosvelGapSpikeCount(uint8_t node_id);
uint16_t canCopyPosvelGapSpikeTimes(uint8_t node_id, uint32_t *out, uint16_t max_out);
void canNoteBusRead(uint8_t bus);
void canNoteRxOverflow();
CanRuntimeStats canGetRuntimeStats();
void canResetRuntimeStats();

extern CAN_message_t g_last_can_msg;
void handleCANMessage(const CAN_message_t &msg, uint8_t rx_bus);

#endif
