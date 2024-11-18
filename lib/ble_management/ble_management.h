#pragma once

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "common.h"

extern QueueHandle_t beaconRawData_Q;

class MyAdvertisedDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks {
 public:
  void onResult(NimBLEAdvertisedDevice* advertisedDevice) override;
};

void decodeBeaconData(char beacon_data[19], BeaconData_t& decodedData);

void printBLEHex(std::string& serviceData, size_t length);