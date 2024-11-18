#pragma once

#include <Arduino.h>

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
};