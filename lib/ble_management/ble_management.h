#pragma once

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "common.h"
#include "id_management.h"

extern QueueHandle_t beaconRawData_Q;

class MyAdvertisedDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks {
 public:
  void onResult(NimBLEAdvertisedDevice* advertisedDevice) override;
};

BeaconData_t decodeBeaconData(char (&beacon_data)[19]);

void printBLEHex(std::string& serviceData, size_t length);