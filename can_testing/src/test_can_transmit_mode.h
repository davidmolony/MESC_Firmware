#pragma once
#include "supervisor.h"
#include <FlexCAN_T4.h>
#include <Arduino.h>

void test_can_transmit_mode(Supervisor_typedef *sup,
                            FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can1,
                            FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> &can2);
