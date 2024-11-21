#pragma once

#include <Arduino.h>

#define BEACON_DATA_CHAR_SIZE 19

#define RS485_BAUD_RATE     115200
#define RS485_RX_PIN        18
#define RS485_TX_PIN        19
#define PROTOCOL_DEFAULT_ID 0xB0

struct GPSData_t {
  float longitude;
  float latitude;
  char status;
};

struct BeaconData_t {
  std::string ID;
  GPSData_t gps;
  float voltageSupply;
  time_t hourMeter;
  int8_t rssi;
  uint32_t lastSeen;
};