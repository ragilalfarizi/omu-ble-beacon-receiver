#include <Arduino.h>
#include <NimBLEDevice.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include <unordered_map>
#include <vector>

#include "ble_management.h"
#include "common.h"

/* GLOBAL VARIABLES */
std::vector<BeaconData_t> detectedDevices;
// std::unordered_map<String, BeaconData_t> beaconDataMap;

/* FORWARD DECLARATION FOR FUNCTIONS */
void BLEReceiver(void* pvParameter);
void RS485Comm(void* pvParameter);
void dataProcessing(void* pvParameter);
// void printBLEHex(std::string& serviceData, size_t length);

/* TASK HANDLER DECLARATION */
TaskHandle_t BLEHandler            = NULL;
TaskHandle_t dataProcessingHandler = NULL;
TaskHandle_t RS485Handler          = NULL;

/* QUEUES AND SEMAPHORE DECLARATION */
QueueHandle_t beaconRawData_Q;

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
  xTaskCreatePinnedToCore(dataProcessing, "Data Processing", 4096, NULL, 3,
                          &dataProcessingHandler, 1);
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
  char buffer[19];
  BeaconData_t data;

  while (1) {
    if (xQueueReceive(beaconRawData_Q, buffer, pdMS_TO_TICKS(100)) == pdPASS) {
      // TODO: Decode
      data = decodeBeaconData(buffer);

      // TODO: add to unordered map. (NO DUPLICATE. UPDATE IF THE SAME KEY EXIST
      // BUT THE REST OF DATA IS CHANGING)

      Serial.printf("==================\n");
      Serial.printf(
          "Beacon ID: %s\n",
          data.ID.c_str());  // Print the ID (String -> c_str() for printf)
      Serial.printf("Voltage Supply: %.2f V\n",
                    data.voltageSupply);  // Print voltage supply as float
      Serial.printf("GPS Status: %c\n",
                    data.gps.status);  // Print GPS status as integer
      Serial.printf(
          "Longitude: %.6f\n",
          data.gps.longitude);  // Print longitude as float with 6 decimals
      Serial.printf(
          "Latitude: %.6f\n",
          data.gps.latitude);  // Print latitude as float with 6 decimals
      Serial.printf("Hour Meter: %lu\n",
                    data.hourMeter);  // Print hour meter as unsigned long
      Serial.printf("==================\n");
    } else {
      Serial.println("Beacon Raw Data Queue is empty");
    }
  }
}
