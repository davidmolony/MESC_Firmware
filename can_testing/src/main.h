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
#define CAN_STB         21
#define CAN_TX          24
#define CAN_RX          25



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
