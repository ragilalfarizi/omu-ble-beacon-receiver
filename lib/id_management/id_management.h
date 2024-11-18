#pragma once

#include <Arduino.h>

#include "common.h"

enum ListID_t {
  AC = 0,
  CC,
  CD,
  CE,
  CG,
  CO,
  CT,
  DP,
  FL,
  LS,
  MC,
  MS,
  PP,
  ST,
  TH,
  TL,
  WT,
  XCD,
  XCE,
  XCT,
  XDP,
  XMC,
  XST,
  XWT,
  UNKNOWN,
};

ListID_t stringToEnum(const std::string &id);

// Function to process the input string
bool extractBeaconID(const std::string &input, ListID_t &id, uint16_t &number);