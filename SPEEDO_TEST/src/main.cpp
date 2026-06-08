#include <Arduino.h>
#include <FlexCAN_T4.h>

namespace {

constexpr uint16_t CAN_ID_SPEED = 0x2A0;
constexpr uint16_t CAN_ID_BUS_VOLT_CURR = 0x2B1;
constexpr uint16_t CAN_ID_TEMP_MOT_MOS1 = 0x2B4;
constexpr uint16_t CAN_ID_TEMP_MOS2_MOS3 = 0x2B5;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> canBus;

uint16_t unpackMescMessageId(uint32_t extId) {
  return static_cast<uint16_t>(extId >> 16);
}

uint8_t unpackMescSender(uint32_t extId) {
  return static_cast<uint8_t>(extId & 0xFF);
}

uint8_t unpackMescReceiver(uint32_t extId) {
  return static_cast<uint8_t>((extId >> 8) & 0xFF);
}

float unpackFloatLE(const uint8_t* data) {
  float value = 0.0f;
  memcpy(&value, data, sizeof(float));
  return value;
}

void printCommonHeader(uint16_t msgId, uint8_t sender, uint8_t receiver) {
  Serial.print("id=0x");
  Serial.print(msgId, HEX);
  Serial.print(" sender=");
  Serial.print(sender);
  Serial.print(" receiver=");
  Serial.print(receiver);
  Serial.print(" -> ");
}

void decodeMescTelemetry(const CAN_message_t& msg) {
  if (!msg.flags.extended || msg.len < 8) {
    return;
  }

  const uint16_t mescId = unpackMescMessageId(msg.id);
  const uint8_t sender = unpackMescSender(msg.id);
  const uint8_t receiver = unpackMescReceiver(msg.id);

  const float n1 = unpackFloatLE(&msg.buf[0]);
  const float n2 = unpackFloatLE(&msg.buf[4]);

  switch (mescId) {
    case CAN_ID_SPEED: {
      const float erpm = n1 * 60.0f;
      printCommonHeader(mescId, sender, receiver);
      Serial.print("speed_eHz=");
      Serial.print(n1, 2);
      Serial.print(" speed_erpm=");
      Serial.println(erpm, 1);
      break;
    }
    case CAN_ID_BUS_VOLT_CURR:
      printCommonHeader(mescId, sender, receiver);
      Serial.print("vbus=");
      Serial.print(n1, 2);
      Serial.print("V ibus=");
      Serial.print(n2, 2);
      Serial.println("A");
      break;
    case CAN_ID_TEMP_MOT_MOS1:
      printCommonHeader(mescId, sender, receiver);
      Serial.print("motor_temp=");
      Serial.print(n1, 2);
      Serial.print("C mos1_temp=");
      Serial.print(n2, 2);
      Serial.println("C");
      break;
    case CAN_ID_TEMP_MOS2_MOS3:
      printCommonHeader(mescId, sender, receiver);
      Serial.print("mos2_temp=");
      Serial.print(n1, 2);
      Serial.print("C mos3_temp=");
      Serial.print(n2, 2);
      Serial.println("C");
      break;
    default:
      break;
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {
    // Allow time for USB serial connection on startup.
  }

  canBus.begin();
  canBus.setBaudRate(500000);

  Serial.println("SPEEDO_TEST Teensy 4.0 CAN monitor ready at 500000 bps");
  Serial.println("Expecting MESC extended IDs for speed, vbus/current, and temperatures");
}

void loop() {
  CAN_message_t msg;
  while (canBus.read(msg)) {
    decodeMescTelemetry(msg);
  }
}
