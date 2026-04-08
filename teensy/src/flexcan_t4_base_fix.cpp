#include <FlexCAN_T4.h>

// Workaround for missing FlexCAN_T4_Base::getFirstTxBoxSize() definition
// in the Teensy framework library shipped with PlatformIO.
uint8_t FlexCAN_T4_Base::getFirstTxBoxSize() {
  return 8;
}
