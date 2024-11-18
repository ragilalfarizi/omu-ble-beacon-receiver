#pragma once

#include <Arduino.h>

#define BEACON_DATA_CHAR_SIZE 19

struct GPSData_t {
  double longitude;
  double latitude;
  char status;
};

struct BeaconData_t {
  String ID;
  GPSData_t gps;
  float voltageSupply;
  time_t hourMeter;
  int8_t rssi;
};