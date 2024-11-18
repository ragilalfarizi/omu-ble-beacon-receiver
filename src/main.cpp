#include <Arduino.h>
#include <NimBLEDevice.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include "ble_management.h"

#include <vector>

#include "common.h"

/* GLOBAL VARIABLES */
std::vector<BeaconData_t> detectedDevices;

/* FORWARD DECLARATION FOR FUNCTIONS */
void BLEReceiver(void* pvParameter);
void RS485Comm(void* pvParameter);
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
  // xTaskCreatePinnedToCore(dataProcessing, "Data Processing", 4096, NULL, 3,
                          // &dataProcessingHandler, 0);
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

  while (1) {
    if (xQueueReceive(beaconRawData_Q, buffer, pdMS_TO_TICKS(100)) == pdPASS) {
      // TODO: Decode
      // TODO: add to unordered map. (NO DUPLICATE. UPDATE IF THE SAME KEY EXIST
      // BUT THE REST OF DATA IS CHANGING)
    } else {
      Serial.println("Beacon Raw Data Queue is empty");
    }
  }
}
