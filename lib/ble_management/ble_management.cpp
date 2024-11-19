#include "ble_management.h"

// NimBLE callback to process the received advertisement data
void MyAdvertisedDeviceCallbacks::onResult(
    NimBLEAdvertisedDevice* advertisedDevice) {
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

BeaconData_t decodeBeaconData(char (&beacon_data)[19]) {
  BeaconData_t data;

  /* DECODE ID */
  // Extract List ID (first byte)
  ListID_t listID = static_cast<ListID_t>(beacon_data[0]);
  data.ID         = getListIDString(listID);  // Map List ID to string

  // Extract Beacon Number (next 2 bytes)
  uint16_t beaconNumber = (static_cast<uint16_t>(beacon_data[1]) << 8) |
                          static_cast<uint16_t>(beacon_data[2]);

  // Format the beacon number as a 4-digit string
  std::ostringstream beaconNumberStr;
  beaconNumberStr << std::setw(4) << std::setfill('0') << beaconNumber;

  // Combine List ID and beacon number
  data.ID += beaconNumberStr.str();

  /* DECODE VOLTAGE SUPPLY */
  uint16_t volt = (static_cast<uint16_t>(beacon_data[3]) << 8) |
                  static_cast<uint16_t>(beacon_data[4]);
  data.voltageSupply = static_cast<float>(volt) / 1000.0f;  // Voltage in volts

  /* DECODE GPS STATUS */
  data.gps.status = static_cast<char>(beacon_data[6]);  // GPS status (byte 7)

  /* DECODE GPS LONGITUDE */
  int32_t longitudeFixedPoint = (static_cast<int32_t>(beacon_data[7]) << 24) |
                                (static_cast<int32_t>(beacon_data[8]) << 16) |
                                (static_cast<int32_t>(beacon_data[9]) << 8) |
                                static_cast<int32_t>(beacon_data[10]);
  data.gps.longitude = static_cast<float>(longitudeFixedPoint) / 256.0f;

  /* DECODE GPS LATITUDE */
  int32_t latitudeFixedPoint = (static_cast<int32_t>(beacon_data[11]) << 24) |
                               (static_cast<int32_t>(beacon_data[12]) << 16) |
                               (static_cast<int32_t>(beacon_data[13]) << 8) |
                               static_cast<int32_t>(beacon_data[14]);
  data.gps.latitude = static_cast<float>(latitudeFixedPoint) / 256.0f;

  /* DECODE HOUR METER */
  data.hourMeter = (static_cast<uint32_t>(beacon_data[15]) << 24) |
                   (static_cast<uint32_t>(beacon_data[16]) << 16) |
                   (static_cast<uint32_t>(beacon_data[17]) << 8) |
                   static_cast<uint32_t>(beacon_data[18]);

  return data;
}

void printBLEHex(std::string& serviceData, size_t length) {
  Serial.print("[BLE] Service Data (Hex): ");
  for (size_t i = 0; i < length; i++) {
    Serial.printf("%02X ", (uint8_t)serviceData[i]);
  }
  Serial.println();
}

void printBeaconData(const BeaconData_t& data) {
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
  Serial.printf("Latitude: %.6f\n",
                data.gps.latitude);  // Print latitude as float with 6 decimals
  Serial.printf("Hour Meter: %lu\n",
                data.hourMeter);  // Print hour meter as unsigned long
  Serial.printf("==================\n");
}