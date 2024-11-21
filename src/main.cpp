#include <Arduino.h>
#include <NimBLEDevice.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include <string>
#include <unordered_map>
#include <vector>

#include "ble_management.h"
#include "common.h"
#include "ProtocolAA55.h"
#include "gps.h"

/* GLOBAL VARIABLES */
// std::vector<BeaconData_t> detectedDevices;
std::unordered_map<std::string, BeaconData_t> beaconDataMap;

/* FORWARD DECLARATION FOR FUNCTIONS */
void BLEReceiver(void* pvParameter);
void RS485Comm(void* pvParameter);
void dataProcessing(void* pvParameter);
void printBeaconDataMap(void* pvParameter);
void cleanupBeaconDataMap(void* pvParameter);
// void printBLEHex(std::string& serviceData, size_t length);

/* TASK HANDLER DECLARATION */
TaskHandle_t BLEHandler                = NULL;
TaskHandle_t dataProcessingHandler     = NULL;
TaskHandle_t RS485Handler              = NULL;
TaskHandle_t printBeaconDataMapHandler = NULL;
TaskHandle_t cleanupDataMapHandler     = NULL;

/* QUEUES AND SEMAPHORE DECLARATION */
QueueHandle_t beaconRawData_Q;

void setup() {
  /* SERIAL INIT */
  Serial.begin(9600);

  /* QUEUES AND SEMAPHORE INIT */
  beaconRawData_Q =
      xQueueCreate(10, sizeof(char) * BEACON_DATA_CHAR_SIZE + sizeof(int8_t));
  if (beaconRawData_Q == nullptr) {
    Serial.println("[Error] Failed to create BLE data queue!");
    return;
  }

  xTaskCreatePinnedToCore(BLEReceiver, "BLE Receiver", 4096, NULL, 3,
                          &BLEHandler, 0);
  xTaskCreatePinnedToCore(dataProcessing, "Data Processing", 4096, NULL, 3,
                          &dataProcessingHandler, 1);
  xTaskCreatePinnedToCore(printBeaconDataMap, "print Beacon Map", 4096, NULL, 3,
                          &printBeaconDataMapHandler, 1);
  xTaskCreatePinnedToCore(cleanupBeaconDataMap, "clean up Beacon Map", 4096,
                          NULL, 3, &printBeaconDataMapHandler, 1);
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

void dataProcessing(void* pvParameter) {
  char buffer[BEACON_DATA_CHAR_SIZE + sizeof(int8_t)];
  BeaconData_t data;

  while (1) {
    if (xQueueReceive(beaconRawData_Q, buffer, pdMS_TO_TICKS(100)) == pdPASS) {
      data = decodeBeaconData(buffer, sizeof(buffer));

      // UNCOMMENT TO PRINT DEBUG
      // printBeaconData(data);

      // Get the current time in milliseconds
      uint32_t currentTime = millis();
      data.lastSeen        = currentTime;

      // TODO: add to unordered map. (NO DUPLICATE. UPDATE IF THE SAME KEY EXIST
      // BUT THE REST OF DATA IS CHANGING)

      // Check if the ID already exists in the unordered map
      auto it = beaconDataMap.find(data.ID);

      if (it != beaconDataMap.end()) {
        // If found, update the data
        it->second = data;
        Serial.printf("Updated Beacon Data with ID: %s\n", data.ID.c_str());
      } else {
        // If not found, insert the new data
        beaconDataMap[data.ID] = data;
        Serial.printf("Inserted new Beacon Data with ID: %s\n",
                      data.ID.c_str());
      }

    } else {
      Serial.println("Beacon Raw Data Queue is empty");
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void printBeaconDataMap(void* pvParameter) {
  while (1) {
    // Print the number of objects in the map
    Serial.printf("Total Beacon Data entries: %d\n", beaconDataMap.size());

    // Iterate over the unordered map and print each entry
    Serial.println("=============================================");
    for (const auto& entry : beaconDataMap) {
      Serial.printf(
          "ID: %s, Voltage: %.2f, GPS Status: %c, Longitude: %.6f, Latitude: "
          "%.6f, Hour Meter: %d, RSSI: %d\n",
          entry.second.ID.c_str(), entry.second.voltageSupply,
          entry.second.gps.status, entry.second.gps.longitude,
          entry.second.gps.latitude, entry.second.hourMeter, entry.second.rssi);
    }
    Serial.println("=============================================");

    // Delay for a while before printing again (e.g., 5 seconds)
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void cleanupBeaconDataMap(void* pvParameter) {
  const uint32_t timeoutInterval = 10000;  // 10 seconds
  while (1) {
    uint32_t currentTime = millis();

    // Iterate over the map and remove stale entries
    for (auto it = beaconDataMap.begin(); it != beaconDataMap.end();) {
      if ((currentTime - it->second.lastSeen) > timeoutInterval) {
        Serial.printf("Removing stale Beacon Data with ID: %s\n",
                      it->second.ID.c_str());
        it = beaconDataMap.erase(it);  // Remove and advance iterator
      } else {
        ++it;  // Advance iterator
      }
    }

    // Delay the cleanup task (e.g., run every 5 seconds)
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}
