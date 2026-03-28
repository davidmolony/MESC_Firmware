#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <IntervalTimer.h>
#include <FlexCAN_T4.h>

// ================= MESC CAN definitions =================
#define CAN_ID_IQREQ    0x001
#define TEENSY_NODE_ID  0x03 // sender (is this Teensy)

// ================= Peripherals =================
#define LED1_PIN        2
#define LED2_PIN        3
#define PUSHBUTTON_PIN  19
#define SPEAKER_PIN     14
#define CS_PIN          10
#define INT_PIN         20
#define CAN_STB         21   // Verify this pin drives the CAN2 transceiver standby on brain_board V1.6.
// FlexCAN_T4 setRX()/setTX() takes FLEXCAN_PINS enum, not literal GPIO numbers.
// For CAN1 on Teensy 4.0, DEF selects CAN1_RX=pin 23 and CAN1_TX=pin 22.
#define CAN1_PINSEL     DEF
// Set to 0 to fully disable CAN1 traffic during A/B tests (TX and RX on Teensy side).
#define CAN1_ENABLE     1
// For CAN2 on Teensy 4.0, DEF selects CAN2_RX=pin 0 and CAN2_TX=pin 1.
#define CAN2_PINSEL     DEF

// ================= CAN TX routing =================
// Route each ESC node to exactly one physical CAN controller.
// This avoids writing every command on both buses.
#define CAN_TX_BUS_CAN1 1
#define CAN_TX_BUS_CAN2 2
#define CAN_TX_BUS_BOTH 3

// Current bench mapping:
// - node 11 uses CAN2
// - node 12 uses CAN1
#define ESC_NODE11_TX_BUS CAN_TX_BUS_CAN2
#define ESC_NODE12_TX_BUS CAN_TX_BUS_CAN1

static inline uint8_t can_tx_bus_for_node(uint8_t node_id) {
  if (node_id == 11u) return ESC_NODE11_TX_BUS;
  if (node_id == 12u) return ESC_NODE12_TX_BUS;
  return CAN_TX_BUS_BOTH;  // fallback for unknown nodes
}



// ================= RC PWM channels =================
#define RC_INPUT1       9
#define RC_INPUT2       8
#define RC_INPUT3       7
#define RC_INPUT4       6
#define RC_INPUT5       5
#define RC_INPUT6       4

// Non-blocking short beeper chirp (uses TonePlayer state machine)
void beeper_tweet();

#endif // MAIN_H
