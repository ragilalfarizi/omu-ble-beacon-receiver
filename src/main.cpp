#include <Arduino.h>
#include <NimBLEDevice.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include <string>
#include <unordered_map>
#include <vector>

#include "ProtocolAA55.h"
#include "ble_management.h"
#include "common.h"
#include "gps.h"

/* GLOBAL VARIABLES */
std::unordered_map<std::string, BeaconData_t> beaconDataMap;
GPS* gps;
GPSData_t masterGPS;
ProtocolAA55 protocol(&Serial1, PROTOCOL_DEFAULT_ID);
// HardwareSerial RS485(1);

/* FORWARD DECLARATION FOR FUNCTIONS */
void BLEReceiver(void* pvParameter);
void RS485Comm(void* pvParameter);
void dataProcessing(void* pvParameter);
void printBeaconDataMap(void* pvParameter);
void cleanupBeaconDataMap(void* pvParameter);
void retrieveGPSData(void* pvParam);
// void printBLEHex(std::string& serviceData, size_t length);

/* TASK HANDLER DECLARATION */
TaskHandle_t BLEHandler                = NULL;
TaskHandle_t dataProcessingHandler     = NULL;
TaskHandle_t RS485Handler              = NULL;
TaskHandle_t printBeaconDataMapHandler = NULL;
TaskHandle_t cleanupDataMapHandler     = NULL;
TaskHandle_t retrieveGPSHandler        = NULL;

/* QUEUES AND SEMAPHORE DECLARATION */
QueueHandle_t beaconRawData_Q;
SemaphoreHandle_t beaconDataMutex;

void setup() {
  /* SERIAL INIT */
  Serial.begin(9600);

  /* GPS INIT */
  Serial.println("[GPS] Inisialisasi GPS");
  gps                 = new GPS();
  masterGPS.latitude  = 0;
  masterGPS.longitude = 0;
  masterGPS.status    = 'V';

  /* RS485 INIT */
  // RS485.begin(RS485_BAUD_RATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  if (protocol.begin(RS485_BAUD_RATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN,
                     false)) {
    Serial.println("ProtocoalAA55 has been initialized");
  } else {
    Serial.println("ProtocoalAA55 failed to initialize");
  }

  /* QUEUES AND SEMAPHORE INIT */
  beaconRawData_Q =
      xQueueCreate(10, sizeof(char) * BEACON_DATA_CHAR_SIZE + sizeof(int8_t));
  if (beaconRawData_Q == nullptr) {
    Serial.println("[Error] Failed to create BLE data queue!");
    return;
  }

  beaconDataMutex = xSemaphoreCreateMutex();
  if (beaconDataMutex == NULL) {
    Serial.println("Failed to create mutex");
    return;
  }
  xSemaphoreGive(beaconDataMutex);

  xTaskCreatePinnedToCore(BLEReceiver, "BLE Receiver", 4096, NULL, 3,
                          &BLEHandler, 0);
  xTaskCreatePinnedToCore(retrieveGPSData, "get GPS Data", 4096, NULL, 4,
                          &retrieveGPSHandler, 0);
  xTaskCreatePinnedToCore(dataProcessing, "Data Processing", 4096, NULL, 5,
                          &dataProcessingHandler, 1);
  xTaskCreatePinnedToCore(printBeaconDataMap, "print Beacon Map", 4096, NULL, 3,
                          &printBeaconDataMapHandler, 1);
  xTaskCreatePinnedToCore(cleanupBeaconDataMap, "clean up Beacon Map", 4096,
                          NULL, 2, &printBeaconDataMapHandler, 1);
  xTaskCreatePinnedToCore(RS485Comm, "RS485 Comm", 4096, NULL, 4, &RS485Handler,
                          1);
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
  // std::vector<BeaconData_t> beaconVector;
  size_t size;

  while (1) {
    // TODO: Add mutex or queue
    if (xSemaphoreTake(beaconDataMutex, portMAX_DELAY) == pdTRUE) {
      // Get GPS Data
      // masterGPS.latitude  = gps->location.lat();
      // masterGPS.longitude = gps->location.lng();

      // Convert Unordered Map into vector
      size = beaconDataMap.size();
      BeaconData_t listDetectedBeacon[size];

      // Copy the map values into the listDetectedBeacon array using a regular
      // for loop
      size_t index = 0;
      for (auto it = beaconDataMap.begin(); it != beaconDataMap.end(); ++it) {
        listDetectedBeacon[index] =
            it->second;  // Assign each BeaconData_t from map to the array
        index++;
      }

      // Print the listDetectedBeacon array before sending data
      Serial.println("--------------------");
      Serial.println("List of detected beacons:");
      for (size_t i = 0; i < size; i++) {
        Serial.printf("Beacon ID: %s\n", listDetectedBeacon[i].ID.c_str());
        Serial.printf("GPS Status: %c\n", listDetectedBeacon[i].gps.status);
        Serial.printf("GPS Longitude: %f\n",
                      listDetectedBeacon[i].gps.longitude);
        Serial.printf("GPS Latitude: %f\n", listDetectedBeacon[i].gps.latitude);
        Serial.printf("Hour Meter: %ld\n", listDetectedBeacon[i].hourMeter);
        Serial.printf("RSSI: %d\n", listDetectedBeacon[i].rssi);
        Serial.println("--------------------");
      }

      // Send packet data
      protocol.SendDataBeacon(size, masterGPS, listDetectedBeacon);
      Serial.println("Data is sent through RS485");

      xSemaphoreGive(beaconDataMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void dataProcessing(void* pvParameter) {
  char buffer[BEACON_DATA_CHAR_SIZE + sizeof(int8_t)];
  BeaconData_t data;

  while (1) {
    if (xSemaphoreTake(beaconDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (xQueueReceive(beaconRawData_Q, buffer, pdMS_TO_TICKS(100)) ==
          pdPASS) {
        data = decodeBeaconData(buffer, sizeof(buffer));

        // UNCOMMENT TO PRINT DEBUG
        // printBeaconData(data);

        // Get the current time in milliseconds
        uint32_t currentTime = millis();
        data.lastSeen        = currentTime;

        // Check if the ID already exists in the unordered map
        auto it = beaconDataMap.find(data.ID);

        if (it != beaconDataMap.end()) {
          // If found, update the data
          it->second = data;
          // Serial.printf("Updated Beacon Data with ID: %s\n",
          // data.ID.c_str());
        } else {
          // If not found, insert the new data
          beaconDataMap[data.ID] = data;
          // Serial.printf("Inserted new Beacon Data with ID: %s\n",
          //               data.ID.c_str());
        }

      } else {
        // Serial.println("Beacon Raw Data Queue is empty");
      }

      xSemaphoreGive(beaconDataMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void printBeaconDataMap(void* pvParameter) {
  while (1) {
    if (xSemaphoreTake(beaconDataMutex, portMAX_DELAY) == pdTRUE) {
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
            entry.second.gps.latitude, entry.second.hourMeter,
            entry.second.rssi);
      }
      Serial.println("=============================================");

      xSemaphoreGive(beaconDataMutex);
    }
    // Delay for a while before printing again (e.g., 5 seconds)
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void cleanupBeaconDataMap(void* pvParameter) {
  const uint32_t timeoutInterval = 10000;  // 10 seconds

  while (1) {
    if (xSemaphoreTake(beaconDataMutex, portMAX_DELAY) == pdTRUE) {
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

      xSemaphoreGive(beaconDataMutex);
    }

    // Delay the cleanup task (e.g., run every 5 seconds)
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void retrieveGPSData(void* pvParam) {
  // bool isValid = false;

  while (1) {
    Serial.println("[GPS] encoding...");

    while (Serial.available() > 0) {
      char gpsChar = Serial.read();
      gps->encode(gpsChar);
    }

    // isValid = gps->getValidation();

    if ((gps->getCharProcessed()) < 10) {
      Serial.println(
          "[GPS] GPS module not sending data, check wiring or module power");
      masterGPS.status = 'V';
    } else {
      if (gps->location.isUpdated()) {
        float latitude  = static_cast<float>(gps->location.lat());
        float longitude = static_cast<float>(gps->location.lng());

        Serial.printf("[GPS] Latitude : %f\n", latitude);
        Serial.printf("[GPS] Longitude : %f\n", longitude);

        masterGPS.latitude = latitude;
        masterGPS.latitude = longitude;
        masterGPS.status   = 'A';
      } else {
        Serial.println("[GPS] GPS is searching for a signal...");
        masterGPS.status = 'V';
      }
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}