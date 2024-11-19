#pragma once

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <iomanip>
#include <sstream>
#include <string>

#include "common.h"
#include "id_management.h"

extern QueueHandle_t beaconRawData_Q;

class MyAdvertisedDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks {
 public:
  void onResult(NimBLEAdvertisedDevice* advertisedDevice) override;
};

BeaconData_t decodeBeaconData(const char* beacon_data, size_t size);

void printBLEHex(std::string& serviceData, size_t length);

void printBeaconData(const BeaconData_t& data);