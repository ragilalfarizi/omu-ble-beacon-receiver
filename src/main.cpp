#include <Arduino.h>
#include <NimBLEDevice.h>

#include <vector>

#include "common.h"

// int16_t analogInputFixedPoint = 0;  // Stores the fixed-point value received
// float analogInputVal =
//     0.0;  // Stores the floating-point value converted from fixed-point

/* GLOBAL VARIABLES */
std::vector<BeaconData_t> detectedDevices;

/* FORWARD DECLARATION FOR FUNCTIONS */
void BLEReceiver(void* pvParameter);
void RS485Comm(void* pvParameter);

/* TASK HANDLER DECLARATION */
TaskHandle_t BLEHandler   = NULL;
TaskHandle_t RS485Handler = NULL;

void decodeBeaconData(char beacon_data[19], BeaconData_t& decodedData) {
  // Decode the voltage supply
  uint16_t volt = ((uint16_t)beacon_data[3] << 8) | (uint16_t)beacon_data[4];
  decodedData.voltageSupply = volt / 1000.0;  // Convert from mV to volts

  // Decode the GPS status
  decodedData.gps.status = beacon_data[6];

  // Decode the longitude
  int32_t longitudeFixedPoint =
      ((int32_t)beacon_data[7] << 24) | ((int32_t)beacon_data[8] << 16) |
      ((int32_t)beacon_data[9] << 8) | (int32_t)beacon_data[10];
  decodedData.gps.longitude = longitudeFixedPoint / 256.0;  // Convert to double

  // Decode the latitude
  int32_t latitudeFixedPoint =
      ((int32_t)beacon_data[11] << 24) | ((int32_t)beacon_data[12] << 16) |
      ((int32_t)beacon_data[13] << 8) | (int32_t)beacon_data[14];
  decodedData.gps.latitude = latitudeFixedPoint / 256.0;  // Convert to double

  // Decode the hour meter (4 bytes)
  decodedData.hourMeter =
      ((uint32_t)beacon_data[15] << 24) | ((uint32_t)beacon_data[16] << 16) |
      ((uint32_t)beacon_data[17] << 8) | (uint32_t)beacon_data[18];
}

// NimBLE callback to process the received advertisement data
class MyAdvertisedDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
    if (advertisedDevice->haveServiceData()) {
      // Get service data
      std::string serviceData = advertisedDevice->getServiceData();

      // Ensure the service data is the correct size (15 bytes in your case)
      if (serviceData.length() == 19) {
        // Copy the service data into a char array for decoding
        char beacon_data[19];
        memcpy(beacon_data, serviceData.data(), 19);
        Serial.println(beacon_data);

        // Create a BeaconData_t structure to hold the decoded values
        BeaconData_t data;

        // Call your decode function to extract the data
        decodeBeaconData(beacon_data, data);

        // Print out the decoded data
        Serial.printf("============================================\n");
        Serial.printf("GPS STATUS\t\t= %c\n", data.gps.status);
        Serial.printf("GPS LATITUDE\t\t= %.6f\n", data.gps.latitude);
        Serial.printf("GPS LONGITUDE\t\t= %.6f\n", data.gps.longitude);
        Serial.printf("Analog Input\t\t= %.2f V\n", data.voltageSupply);
        Serial.printf("Hour Meter\t\t= %ld s\n", data.hourMeter);
        Serial.printf("============================================\n");
      } else {
        Serial.println("Invalid service data size");
      }
    }
  }
};

void setup() {
  Serial.begin(9600);

  xTaskCreatePinnedToCore(BLEReceiver, "BLE Receiver", 4096, NULL, 3,
                          &BLEHandler, 0);
  // xTaskCreatePinnedToCore(RS485Comm, "RS485 Comm", 4096, NULL, 3,
  // &RS485Handler, 1);
}

void loop() {}

void BLEReceiver(void* pvParameter) {
  // Initialize NimBLE
  NimBLEDevice::init("");

  // Create a BLE scan object
  NimBLEScan* pBLEScan = NimBLEDevice::getScan();

  // Set the callback for processing the received advertising data
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(),
                                         true);

  // Start scanning for BLE devices
  pBLEScan->setActiveScan(true);
  pBLEScan->start(30, false);  // Scanning for 30 seconds

  while (1) {
    // do nothing perhaps?
  }
}

void RS485Comm(void* pvParameter) {
  // Initialize RS485

  while (1) {
    // do nothing perhaps?
  }
}