#include <Arduino.h>
#include <NimBLEDevice.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include <vector>

#include "common.h"

/* GLOBAL VARIABLES */
std::vector<BeaconData_t> detectedDevices;

/* FORWARD DECLARATION FOR FUNCTIONS */
void BLEReceiver(void* pvParameter);
void RS485Comm(void* pvParameter);
static void printBLEHex(std::string& serviceData, size_t length);

/* TASK HANDLER DECLARATION */
TaskHandle_t BLEHandler   = NULL;
TaskHandle_t RS485Handler = NULL;

/* QUEUES AND SEMAPHORE DECLARATION */
QueueHandle_t beaconRawData_Q;

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
  void onResult(NimBLEAdvertisedDevice* advertisedDevice) override {
    // Filter by Service UUID
    if (!advertisedDevice->isAdvertisingService(BLEUUID((uint16_t)0xFEAA))) {
      return;  // Ignore non-matching services
    }

    // Check for service data
    if (advertisedDevice->haveServiceData()) {
      std::string serviceData = advertisedDevice->getServiceData();

      // Ensure service data length matches expected
      if (serviceData.length() == BEACON_DATA_CHAR_SIZE) {
        char beacon_data[BEACON_DATA_CHAR_SIZE];
        memcpy(beacon_data, serviceData.data(), BEACON_DATA_CHAR_SIZE);

        // UNCOMMENT TO DEBUG
        // printBLEHex(serviceData, serviceData.length());

        // Send to Queue
        if (xQueueSend(beaconRawData_Q, &beacon_data, pdMS_TO_TICKS(100)) ==
            pdPASS) {
          Serial.println("Raw BLE Data is sent to Queue");
        } else {
          Serial.println("Raw BLE Data Queue is full");
        }

      } else {
        Serial.printf("[BLE] Invalid service data size: %d bytes\n",
                      serviceData.length());
      }
    } else {
      Serial.println("[BLE] No service data found in this advertisement.");
    }
  }
};

void setup() {
  /* SERIAL INIT */
  Serial.begin(9600);

  /* QUEUES AND SEMAPHORE INIT */
  beaconRawData_Q = xQueueCreate(10, sizeof(char) * BEACON_DATA_CHAR_SIZE);
  if (beaconRawData_Q == nullptr) {
    Serial.println("[Error] Failed to create BLE data queue!");
    return;
  }

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
  // Configure scanning options
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);  // Scanning interval (in milliseconds)
  pBLEScan->setWindow(50);     // Scanning window (in milliseconds)

  // Start scanning in continuous mode
  pBLEScan->start(0, nullptr);  // 0 = Scan indefinitely

  while (1) {
    // The task runs indefinitely while BLE scanning happens in the background
    vTaskDelay(pdMS_TO_TICKS(1000));  // Optional delay to free CPU time
  }
}

void RS485Comm(void* pvParameter) {
  // Initialize RS485

  while (1) {
    // do nothing perhaps?
  }
}

static void printBLEHex(std::string& serviceData, size_t length) {
  Serial.print("[BLE] Service Data (Hex): ");
  for (size_t i = 0; i < length; i++) {
    Serial.printf("%02X ", (uint8_t)serviceData[i]);
  }
  Serial.println();
}
