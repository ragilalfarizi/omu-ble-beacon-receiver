/**
 *
 *  This libarary is only for supporting some devices which later will connect
 * with RS485 bus line of QIm protocol Nothing special from the created protocol
 * just a common one, construct with header and tail.
 *
 *  Since the connection need to wire as a multidrop device then ID destination
 * and ID source where the message come from are added into protocol packet data
 *
 *  VERSION V.1.0.2
 *
 *  log version (supposed to be defined somehow here) :O
 *  V.1.0.0 --> inlcude function to listen some busy line of RS485, this is
 * dedicated for LCD display module V.1.0.1 --> some bug fix V.1.0.2 --> inlcude
 * function to construct a beacon packet data this is dedicated for Beacon
 * display module
 *
 *
 *  BDO 2024
 *  created: by Rteam
 */

#include "ProtocolAA55.h"

/**
 * @brief creating object for Protocol AA55
 */
ProtocolAA55::ProtocolAA55(HardwareSerial *serial, uint8_t MyOwnIDDev) {
  _hardwareSerial = serial;
  _myID           = MyOwnIDDev;
}

/**
 * @brief intialization function for object of Procol AA55
 * @return status if it success to initialize or not
 */
bool ProtocolAA55::begin(unsigned long baud, uint32_t config, int8_t rxPin,
                         int8_t txPin, bool invert) {
  if (_hardwareSerial) {
    if ((rxPin != -1) && (txPin != -1)) {
      _hardwareSerial->begin(baud, config, rxPin, txPin, invert);
    }

    return true;
  }

  return false;
}

/**
 * @brief intialization for pin RS4485 direction when it need to be controll
 * manually
 * @return -
 */
void ProtocolAA55::configDirPinForRS485(int8_t pinDir) {
  if (pinDir != -1) {
    _dirPin = pinDir;
    pinMode(_dirPin, OUTPUT);
    digitalWrite(_dirPin, LOW);
  }
}

/**
 * @brief intialization function for object of Procol AA55
 * @return -
 */
void ProtocolAA55::configListeningReference(uint8_t _ListeningReference) {
  _listeningModeOnly = true;
  _listeningRef      = _ListeningReference;
}

/**
 * @brief function to check which data should be taken,
 *        for the information there will be some byte is found got an additional
 * byte just to differetiate with tail and header assigment.
 * @return actual byte value
 */
uint8_t ProtocolAA55::checkHeaderValueMiddleData(uint8_t *buffPacket,
                                                 uint8_t *idx) {
  uint8_t temp = 0x00;
  if (buffPacket[(*idx)] == identifyDiffHeaderandData) {
    if (buffPacket[(*idx) + 1] == identifyDiffHeaderandData) {
      (*idx) = (*idx) + 2;
      return identifyDiffHeaderandData;
    } else {
      (*idx) = (*idx) + 2;
      temp   = buffPacket[(*idx) - 1];
      // Serial.printf(" $%02X", temp);
      return temp;
    }
  } else if (buffPacket[(*idx)] == identifyDiffTailData) {
    if (buffPacket[(*idx) + 1] == identifyDiffTailData) {
      (*idx) = (*idx) + 2;
      return identifyDiffTailData;
    } else {
      (*idx) = (*idx) + 2;
      temp   = buffPacket[(*idx) - 1];
      // Serial.printf(" $%02X", temp);
      return temp;
    }
  } else {
    (*idx) = (*idx) + 1;
    temp   = buffPacket[(*idx) - 1];
    // Serial.printf(" $%02X", temp);
    return temp;
  }
}

/**
 * @brief function to check which data should be added an additional byte to
 * differentiate with header and tail assingment
 * @return
 */
uint8_t ProtocolAA55::addIdenfierWhenHeaderIsData(uint8_t currValue,
                                                  uint8_t *buff, uint8_t *idx) {
  if ((currValue == 0xAA) || (currValue == identifyDiffHeaderandData)) {
    buff[(*idx) + 1] = identifyDiffHeaderandData;
    (*idx)           = (*idx) + 1;
    return currValue;
  } else if ((currValue == 0x55) || (currValue == identifyDiffTailData)) {
    buff[(*idx) + 1] = identifyDiffTailData;
    (*idx)           = (*idx) + 1;
    return currValue;
  } else {
    return currValue;
  }
}

/**
 * @brief checksum calculation fucntion for incoming data packet
 * @return
 */
bool ProtocolAA55::checksum_check(uint8_t startByte, uint8_t *buf,
                                  uint8_t *size_) {
  uint16_t sum_val       = 0x00;
  uint8_t checksum_value = 0x00;
  uint8_t checksum_recv  = 0x00;

  // special condition if we found checksum value is equals with tail and header
  // data
  if ((buf[(*size_) - 3] == identifyDiffHeaderandData) ||
      (buf[(*size_) - 3] == identifyDiffTailData)) {
    for (uint8_t x = startByte; x < ((*size_) - 4); x++) {
      sum_val = sum_val + buf[x];
    }

    checksum_recv = buf[(*size_) - 4];
  } else {
    for (uint8_t x = startByte; x < ((*size_) - 3); x++) {
      sum_val = sum_val + buf[x];
    }
    checksum_recv = buf[(*size_) - 3];
  }

  checksum_value = 0xFF - (sum_val & 0xFF);

#if (DEBUG_INTERNAL_LIB == 1)
  Serial.printf("chksum: %02x %02x\r\n", checksum_value, checksum_recv);
#endif

  if ((buf[(*size_) - 3] == identifyDiffHeaderandData) ||
      (buf[(*size_) - 3] == identifyDiffTailData)) {
    if (checksum_value == buf[(*size_) - 4]) {
      return true;
    } else {
      return false;
    }
  } else {
    if (checksum_value == buf[(*size_) - 3]) {
      return true;
    } else {
      return false;
    }
  }
}

/**
 * @brief function for converting float into byte format which fits with the
 * defined protocol rules
 */
uint32_t ProtocolAA55::changeFloatToBytesFormat(float *value_) {
  if ((*value_) < 0) {
    return (uint32_t)abs((*value_) * 10000.0) | 0x80000000;
  } else {
    return (uint32_t)abs((*value_) * 10000.0);
  }
}

/**
 * @brief checksum calculation fucntion for data packet which later will be send
 * to it's destination
 * @return
 */
void ProtocolAA55::checksum_calc(uint8_t offsetByte, uint8_t *buf,
                                 uint8_t *size_) {
  uint16_t sum_val = 0x00;
  uint8_t last_idx = *size_;

  for (uint8_t x = offsetByte; x < ((*size_) + 1); x++) {
    sum_val = sum_val + buf[x];
  }

  buf[++last_idx] = 0xFF - (sum_val & 0xFF);

  // special condition if we found checksum value is equals with tail or header
  // data
  //: CAUTION:
  // perhaps header data combination 1 is not equals with header data
  // combination 2 then we used to check our logic below
  if ((buf[last_idx] == HeaderDataCombination1)) {
    buf[++last_idx] = identifyDiffHeaderandData;
    *size_          = last_idx;
  } else if ((buf[last_idx] == TailDataCombination1)) {
    buf[++last_idx] = identifyDiffTailData;
    *size_          = last_idx;
  } else if ((buf[last_idx] == identifyDiffHeaderandData)) {
    buf[++last_idx] = identifyDiffHeaderandData;
    *size_          = last_idx;
  } else if ((buf[last_idx] == identifyDiffTailData)) {
    buf[++last_idx] = identifyDiffTailData;
    *size_          = last_idx;
  } else {
    *size_ = last_idx;
  }
}

/**
 * @brief function to send bare packet data into serial bus line
 * @return
 */
void ProtocolAA55::SendPayloadUart(uint8_t *buf, uint8_t size_,
                                   uint16_t delayMicrosec) {
  if (_dirPin != -1) {
    digitalWrite(_dirPin, HIGH);
    delayMicroseconds(delayMicrosec);
  }

  _hardwareSerial->write(buf, size_);
  _hardwareSerial->flush();

  if (_dirPin != -1) {
    delayMicroseconds(delayMicrosec);
    digitalWrite(_dirPin, LOW);
  }
}

/**
 * @brief function to listen any data inside bus line and check it either it
 * need or nor this checking is depend on configuration setup in
 * configListeningReference()
 * @return DATA_COMPLETE
 *         DATA_IDLE
 */
uint8_t ProtocolAA55::ListeningData() {
  while (_hardwareSerial->available()) {
    // serial_line_empty = false;
    uint8_t data = _hardwareSerial->read();
    // SerialBusLineIsIdle_2 = false;

#if (DEBUG_INTERNAL_LIB == 1)
    Serial.printf(" %02X|%d ", data, UART_DataLength);
#endif

    if (CollectData) {
      if (FoundFirstTailData) {
        // check length of tail data at the of data
        // if we have more than 3 bytes tail data then currently we only check
        // first tail and the last tail
        if (data == TAIL_DATA_2ND_BYTE && (UART_DataLength > 2)) {
          payloadUart[UART_DataLength] = data;
          UART_DataLength++;
          // uart_evt.evt_type = NEW_DATA_RECEIVED_BY_UART_2;
          // uart_2_StructHandle->uart_2_data_handle(&uart_evt,
          // &payloadUart2[0], &UART2_DataLength);
          CollectData        = false;
          FoundFirstTailData = false;

          return DATA_COMPLETE;
          break;
        } else {
          CollectData        = true;
          FoundFirstTailData = false;
        }
      }

      if (data == TAIL_DATA_1ST_BYTE) {
        FoundFirstTailData = true;
      }

      if (checkPIDData) {
        if (_listeningModeOnly && (_listeningRef & REFERENCE_PID_DATA)) {
          if ((data == PID_DATA_REQ_OIL_OUTGOING) ||
              (data == PID_DATA_STREAMING_OUTGOING) ||
              (data == PID_DATA_TRANSACTION_FULL_OUTGOING) ||
              (data == PID_DATA_TRANSACTION_TRIGGER_OUTGOING) ||
              (data == PID_DATA_REQ_OIL_INCOMING) ||
              (data == PID_DATA_STREAMING_INCOMING) ||
              (data == PID_DATA_TRANSACTION_INCOMING)) {
            CollectData = true;
          } else {
            CollectData = false;
          }
        }
        checkPIDData = false;
      }

      if (checkSourceData) {
        if (_listeningModeOnly && (_listeningRef & REFERENCE_SOURCE_ID)) {
          if ((data == ID_MASTER) || (data == ID_DEV_LMS_1) ||
              (data == ID_DEV_LMS_2) || (data == ID_DEV_LMS_3) ||
              (data == ID_DEV_LMS_4) || (data == ID_DEV_LMS_5) ||
              (data == ID_DEV_LMS_6)) {
            CollectData  = true;
            checkPIDData = true;
          } else {
            CollectData = false;
          }
        }
        checkSourceData = false;
      }
    }

    if (CheckDestinationData) {
      if (_listeningModeOnly && (_listeningRef & REFERENCE_DESTINATION_ID)) {
        if ((data == ID_MASTER) || (data == ID_DEV_LMS_1) ||
            (data == ID_DEV_LMS_2) || (data == ID_DEV_LMS_3) ||
            (data == ID_DEV_LMS_4) || (data == ID_DEV_LMS_5) ||
            (data == ID_DEV_LMS_6)) {
          CollectData     = true;
          checkSourceData = true;
        } else {
          CollectData  = false;
          checkPIDData = false;
        }
      } else {
        if (data == _myID) {
          CollectData = true;
          // checK_request_and_command = true;
          // BypassLengthDataChecking_2 = true;
        } else {
          CollectData = false;
        }
      }
      CheckDestinationData = false;
    }

    if (!CollectData) {
      // implemented for max 2 bytes header
      if (!checkNextHeader) {
        if (data == HEADER_DATA_1ST_BYTE) {
          checkNextHeader = true;
        }
      } else {
        if (data == HEADER_DATA_2ND_BYTE) {
          FoundFirstTailData = false;
          CollectData        = true;
          UART_DataLength    = 0;

          memset(payloadUart, 0, sizeof(payloadUart));
          HeaderBypass         = true;
          CheckDestinationData = true;
        }
        checkNextHeader = false;
      }
    }

    if (CollectData && !HeaderBypass) {
      payloadUart[UART_DataLength] = data;
      UART_DataLength++;
    } else {
      // just bypassing data header before collecting all data
      if (HeaderBypass) {
        HeaderBypass = false;
      }
    }
  }

  SerialBusLineIsIdle = true;

  return DATA_IDLE;
}

/**
 * @brief public function that user use to send packet data into serial bus line
 * @return
 */
void ProtocolAA55::sendDatatoBusLine(uint8_t destinationID, uint8_t *packet,
                                     uint8_t *lenData) {
  uint8_t idxPrint = 0;

  // add some header
  payloadUart[idxPrint] = 0xff;
  // payloadData[++idxPrint]          = 0x00;

  // add distantion ID and its source
  payloadUart[++idxPrint] = 0xAA;
  payloadUart[++idxPrint] = 0xAA;
  payloadUart[++idxPrint] = destinationID;
  payloadUart[++idxPrint] = _myID;

  // generate data packet with data actual
  for (uint8_t xx1 = 0; xx1 < (*lenData); xx1++) {
    payloadUart[++idxPrint] =
        addIdenfierWhenHeaderIsData(packet[xx1], payloadUart, &idxPrint);
  }

  checksum_calc(4, payloadUart, &idxPrint);
  payloadUart[++idxPrint] = 0x55;
  payloadUart[++idxPrint] = 0x55;

  SendPayloadUart(payloadUart, ++idxPrint, 200);
}

/**
 * @brief function to create beacon packet data to match with a rs485 rules
 * @return
 */
void ProtocolAA55::SendDataBeacon(uint8_t totalDetectedBeaocon_,
                                  GPSData_t receiverPosition_,
                                  BeaconData_t listDetecetecBeacon_[]) {
  uint8_t idxArr       = 0;
  uint32_t tempValue   = 0;
  uint8_t tempValue8   = 0;
  uint16_t tempValue16 = 0;

  payloadUart[idxArr]   = 0xff;
  payloadUart[++idxArr] = 0x00;
  payloadUart[++idxArr] = 0x00;
  payloadUart[++idxArr] = 0xA0;
  payloadUart[++idxArr] = _myID;
  payloadUart[++idxArr] = PID_BEACON_DATA;

  payloadUart[++idxArr] = addIdenfierWhenHeaderIsData(
      (receiverPosition_.status), payloadUart, &idxArr);

  tempValue = changeFloatToBytesFormat(&receiverPosition_.longitude);
  payloadUart[++idxArr] = addIdenfierWhenHeaderIsData(
      ((tempValue >> 24) & 0xFF), payloadUart, &idxArr);
  payloadUart[++idxArr] = addIdenfierWhenHeaderIsData(
      ((tempValue >> 16) & 0xFF), payloadUart, &idxArr);
  payloadUart[++idxArr] = addIdenfierWhenHeaderIsData(((tempValue >> 8) & 0xFF),
                                                      payloadUart, &idxArr);
  payloadUart[++idxArr] = addIdenfierWhenHeaderIsData(((tempValue >> 0) & 0xFF),
                                                      payloadUart, &idxArr);

  tempValue             = changeFloatToBytesFormat(&receiverPosition_.latitude);
  payloadUart[++idxArr] = addIdenfierWhenHeaderIsData(
      ((tempValue >> 24) & 0xFF), payloadUart, &idxArr);
  payloadUart[++idxArr] = addIdenfierWhenHeaderIsData(
      ((tempValue >> 16) & 0xFF), payloadUart, &idxArr);
  payloadUart[++idxArr] = addIdenfierWhenHeaderIsData(((tempValue >> 8) & 0xFF),
                                                      payloadUart, &idxArr);
  payloadUart[++idxArr] = addIdenfierWhenHeaderIsData(((tempValue >> 0) & 0xFF),
                                                      payloadUart, &idxArr);

  payloadUart[++idxArr] = addIdenfierWhenHeaderIsData((totalDetectedBeaocon_),
                                                      payloadUart, &idxArr);

  for (uint8_t xx8 = 0; xx8 < totalDetectedBeaocon_; xx8++) {
    tempValue8 = listDetecetecBeacon_[xx8].ID.length();

    payloadUart[++idxArr] =
        addIdenfierWhenHeaderIsData(tempValue8, payloadUart, &idxArr);

    for (uint8_t xx5 = 0; xx5 < tempValue8; xx5++) {
      payloadUart[++idxArr] = addIdenfierWhenHeaderIsData(
          (listDetecetecBeacon_[xx8].ID[xx5]), payloadUart, &idxArr);
    }

    payloadUart[++idxArr] = addIdenfierWhenHeaderIsData(
        (listDetecetecBeacon_[xx8].gps.status), payloadUart, &idxArr);

    tempValue =
        changeFloatToBytesFormat(&listDetecetecBeacon_[xx8].gps.longitude);
    payloadUart[++idxArr] = addIdenfierWhenHeaderIsData(
        ((tempValue >> 24) & 0xFF), payloadUart, &idxArr);
    payloadUart[++idxArr] = addIdenfierWhenHeaderIsData(
        ((tempValue >> 16) & 0xFF), payloadUart, &idxArr);
    payloadUart[++idxArr] = addIdenfierWhenHeaderIsData(
        ((tempValue >> 8) & 0xFF), payloadUart, &idxArr);
    payloadUart[++idxArr] = addIdenfierWhenHeaderIsData(
        ((tempValue >> 0) & 0xFF), payloadUart, &idxArr);

    tempValue =
        changeFloatToBytesFormat(&listDetecetecBeacon_[xx8].gps.latitude);
    payloadUart[++idxArr] = addIdenfierWhenHeaderIsData(
        ((tempValue >> 24) & 0xFF), payloadUart, &idxArr);
    payloadUart[++idxArr] = addIdenfierWhenHeaderIsData(
        ((tempValue >> 16) & 0xFF), payloadUart, &idxArr);
    payloadUart[++idxArr] = addIdenfierWhenHeaderIsData(
        ((tempValue >> 8) & 0xFF), payloadUart, &idxArr);
    payloadUart[++idxArr] = addIdenfierWhenHeaderIsData(
        ((tempValue >> 0) & 0xFF), payloadUart, &idxArr);

    payloadUart[++idxArr] = addIdenfierWhenHeaderIsData(
        ((listDetecetecBeacon_[xx8].hourMeter >> 24) & 0xFF), payloadUart,
        &idxArr);
    payloadUart[++idxArr] = addIdenfierWhenHeaderIsData(
        ((listDetecetecBeacon_[xx8].hourMeter >> 16) & 0xFF), payloadUart,
        &idxArr);
    payloadUart[++idxArr] = addIdenfierWhenHeaderIsData(
        ((listDetecetecBeacon_[xx8].hourMeter >> 8) & 0xFF), payloadUart,
        &idxArr);
    payloadUart[++idxArr] = addIdenfierWhenHeaderIsData(
        ((listDetecetecBeacon_[xx8].hourMeter) & 0xFF), payloadUart, &idxArr);

    if (listDetecetecBeacon_[xx8].rssi < 0) {
      tempValue16 = abs(listDetecetecBeacon_[xx8].rssi * 1) | 0x8000;
    } else {
      tempValue16 = abs(listDetecetecBeacon_[xx8].rssi * 1);
    }
    payloadUart[++idxArr] = addIdenfierWhenHeaderIsData(
        ((tempValue16 >> 8) & 0xFF), payloadUart, &idxArr);
    payloadUart[++idxArr] = addIdenfierWhenHeaderIsData(((tempValue16) & 0xFF),
                                                        payloadUart, &idxArr);
  }
  checksum_calc(4, payloadUart, &idxArr);
  payloadUart[1]        = 0xAA;
  payloadUart[2]        = 0xAA;
  payloadUart[++idxArr] = 0x55;
  payloadUart[++idxArr] = 0x55;

  SendPayloadUart(payloadUart, ++idxArr, 100);
}

/**
 * @brief public function that user use to get a specific data in the bus line
 * communication
 * @return
 */
void ProtocolAA55::getBusData() {
  uint8_t statusData     = DATA_IDLE;
  uint8_t lengthInfoData = 0;
  uint8_t nextIndex      = 0;
  uint8_t unusedDataByte = 0;
  uint16_t unusedData    = 0;
  uint32_t unusedData32  = 0;
  uint8_t indexOilTable  = 0;
  bool enParsingData     = 0;
  statusData             = ListeningData();

  // do some parsing here
  if (statusData == DATA_COMPLETE) {
    if (checksum_check(2, payloadUart, &UART_DataLength)) {
#if (DEBUG_INTERNAL_LIB == 1)
      Serial.println("\nNEW RECEIVE DATA\n");
#endif

      // when master is connected to other devices as a source device
      if (payloadUart[1] == ID_MASTER) {
        if (payloadUart[0] == ID_DEV_LMS_1) {
          enParsingData = true;
          indexOilTable = 0;
        } else if (payloadUart[0] == ID_DEV_LMS_2) {
          enParsingData = true;
          indexOilTable = 1;
        } else if (payloadUart[0] == ID_DEV_LMS_3) {
          enParsingData = true;
          indexOilTable = 2;
        } else if (payloadUart[0] == ID_DEV_LMS_4) {
          enParsingData = true;
          indexOilTable = 3;
        } else if (payloadUart[0] == ID_DEV_LMS_5) {
          enParsingData = true;
          indexOilTable = 4;
        } else if (payloadUart[0] == ID_DEV_LMS_6) {
          enParsingData = true;
          indexOilTable = 5;
        } else {
          enParsingData = false;
        }

        if (enParsingData) {
          // parasing data depend on type of PID Data
          if (payloadUart[2] == PID_DATA_REQ_OIL_OUTGOING) {
            nextIndex = 3;
            // get timestamp
            _OilTable[indexOilTable].timestamp = 0;
            _OilTable[indexOilTable].timestamp =
                (((checkHeaderValueMiddleData(payloadUart, &nextIndex) << 24) |
                  (checkHeaderValueMiddleData(payloadUart, &nextIndex) << 16) |
                  (checkHeaderValueMiddleData(payloadUart, &nextIndex) << 8) |
                  (checkHeaderValueMiddleData(payloadUart, &nextIndex))) &
                 0xFFFFFFFF) +
                (3600 * 7);

#if (DEBUG_INTERNAL_LIB == 1)
            Serial.printf("\nPRS7: %d ", _OilTable[indexOilTable].timestamp);
#endif
            // rawtimeTemp = (time_t)tempValue;
            // timeInfoTemp = localtime(&rawtimeTemp);

            // unused data
            // -------------------------------------------------------------------------------------------------
            unusedData32 =
                ((checkHeaderValueMiddleData(payloadUart, &nextIndex) << 24) |
                 (checkHeaderValueMiddleData(payloadUart, &nextIndex) << 16) |
                 (checkHeaderValueMiddleData(payloadUart, &nextIndex) << 8) |
                 (checkHeaderValueMiddleData(payloadUart, &nextIndex))) &
                0xFFFFFFFF;
            //------------------------------------------------------------------------------------------------------------

            // get lubeman name
            lengthInfoData =
                checkHeaderValueMiddleData(payloadUart, &nextIndex);
            if (lengthInfoData >
                sizeof(_OilTable[indexOilTable].operatorName)) {
              lengthInfoData = sizeof(_OilTable[indexOilTable].operatorName);
            }
#if (DEBUG_INTERNAL_LIB == 1)
            Serial.printf("\nPRS2 %d: ", lengthInfoData);
#endif
            memset(_OilTable[indexOilTable].operatorName, 0,
                   sizeof(_OilTable[indexOilTable].operatorName));
            for (uint8_t xx1 = 0; xx1 < lengthInfoData; xx1++) {
              _OilTable[indexOilTable].operatorName[xx1] =
                  checkHeaderValueMiddleData(payloadUart, &nextIndex);

#if (DEBUG_INTERNAL_LIB == 1)
              Serial.printf("%c ", _OilTable[indexOilTable].operatorName[xx1]);
#endif
            }

// get UnitID
#if (DEBUG_INTERNAL_LIB == 1)
            Serial.printf("\nPRS3 %d: ", lengthInfoData);
#endif
            // get oil type which will be filled in
            lengthInfoData =
                checkHeaderValueMiddleData(payloadUart, &nextIndex);
            if (lengthInfoData >
                sizeof(_OilTable[indexOilTable].unitIDonService)) {
              lengthInfoData = sizeof(_OilTable[indexOilTable].unitIDonService);
            }
            memset(_OilTable[indexOilTable].unitIDonService, 0,
                   sizeof(_OilTable[indexOilTable].unitIDonService));
            for (uint8_t xx1 = 0; xx1 < lengthInfoData; xx1++) {
              _OilTable[indexOilTable].unitIDonService[xx1] =
                  checkHeaderValueMiddleData(payloadUart, &nextIndex);

#if (DEBUG_INTERNAL_LIB == 1)
              Serial.printf("%c ",
                            _OilTable[indexOilTable].unitIDonService[xx1]);
#endif
            }

            // unused data
            // -------------------------------------------------------------------------------------------------
            lengthInfoData =
                checkHeaderValueMiddleData(payloadUart, &nextIndex);
            for (uint8_t xx1 = 0; xx1 < lengthInfoData; xx1++) {
              unusedData = checkHeaderValueMiddleData(payloadUart, &nextIndex);
            }
            //-------------------------------------------------------------------------------------------------------------

            // get HM value of unit
            _OilTable[indexOilTable].HMValueUnit = 0;
            _OilTable[indexOilTable].HMValueUnit =
                (((checkHeaderValueMiddleData(payloadUart, &nextIndex)) << 24) |
                 ((checkHeaderValueMiddleData(payloadUart, &nextIndex)) << 16) |
                 ((checkHeaderValueMiddleData(payloadUart, &nextIndex)) << 8) |
                 (checkHeaderValueMiddleData(payloadUart, &nextIndex))) &
                0xFFFFFFFF;
#if (DEBUG_INTERNAL_LIB == 1)
            Serial.printf("\nPSR4: %d ", _OilTable[indexOilTable].HMValueUnit);
#endif

            // get oil type which will be filled in
            lengthInfoData =
                checkHeaderValueMiddleData(payloadUart, &nextIndex);
            if (lengthInfoData >
                sizeof(_OilTable[indexOilTable].oilNameSpecific)) {
              lengthInfoData = sizeof(_OilTable[indexOilTable].oilNameSpecific);
            }
#if (DEBUG_INTERNAL_LIB == 1)
            Serial.printf("\nPSR5: ");
#endif
            memset(_OilTable[indexOilTable].oilNameSpecific, 0,
                   sizeof(_OilTable[indexOilTable].oilNameSpecific));
            for (uint8_t xx1 = 0; xx1 < lengthInfoData; xx1++) {
              _OilTable[indexOilTable].oilNameSpecific[xx1] =
                  checkHeaderValueMiddleData(payloadUart, &nextIndex);

#if (DEBUG_INTERNAL_LIB == 1)
              Serial.printf("%c ",
                            _OilTable[indexOilTable].oilNameSpecific[xx1]);
#endif
            }

            _OilTable[indexOilTable].ProcessOutgoingOil = 0.0;
            _OilTable[indexOilTable].ProcessIncomingOil = 0.0;
          } else if (payloadUart[2] == PID_DATA_REQ_OIL_INCOMING) {
            // sync time what u get from apps
            nextIndex                          = 2;
            _OilTable[indexOilTable].timestamp = 0;
            _OilTable[indexOilTable].timestamp =
                (((checkHeaderValueMiddleData(payloadUart, &nextIndex) << 24) |
                  (checkHeaderValueMiddleData(payloadUart, &nextIndex) << 16) |
                  (checkHeaderValueMiddleData(payloadUart, &nextIndex) << 8) |
                  (checkHeaderValueMiddleData(payloadUart, &nextIndex))) &
                 0xFFFFFFFF) *
                (3600 * 7);
#if (DEBUG_INTERNAL_LIB == 1)
            Serial.printf("\nPRS7: %d ", _OilTable[indexOilTable].timestamp);
#endif

            // get oil type which will be filled in
            lengthInfoData =
                checkHeaderValueMiddleData(payloadUart, &nextIndex);
            if (lengthInfoData >
                sizeof(_OilTable[indexOilTable].oilNameSpecific)) {
              lengthInfoData = sizeof(_OilTable[indexOilTable].oilNameSpecific);
            }
#if (DEBUG_INTERNAL_LIB == 1)
            Serial.printf("\nPRS8: ");
#endif
            memset(_OilTable[indexOilTable].oilNameSpecific, 0,
                   sizeof(_OilTable[indexOilTable].oilNameSpecific));
            for (uint8_t xx1 = 0; xx1 < lengthInfoData; xx1++) {
              _OilTable[indexOilTable].oilNameSpecific[xx1] =
                  checkHeaderValueMiddleData(payloadUart, &nextIndex);

#if (DEBUG_INTERNAL_LIB == 1)
              Serial.printf("%c ",
                            _OilTable[indexOilTable].oilNameSpecific[xx1]);
#endif
            }

            // get lubeman name
            lengthInfoData =
                checkHeaderValueMiddleData(payloadUart, &nextIndex);
            if (lengthInfoData >
                sizeof(_OilTable[indexOilTable].operatorName)) {
              lengthInfoData = sizeof(_OilTable[indexOilTable].operatorName);
            }
#if (DEBUG_INTERNAL_LIB == 1)
            Serial.printf("\nPRS9: ");
#endif
            memset(_OilTable[indexOilTable].operatorName, 0,
                   sizeof(_OilTable[indexOilTable].operatorName));
            for (uint8_t xx1 = 0; xx1 < lengthInfoData; xx1++) {
              _OilTable[indexOilTable].operatorName[xx1] =
                  checkHeaderValueMiddleData(payloadUart, &nextIndex);

#if (DEBUG_INTERNAL_LIB == 1)
              Serial.printf("%c ", _OilTable[indexOilTable].operatorName[xx1]);
#endif
            }

            _OilTable[indexOilTable].ProcessOutgoingOil = 0.0;
            _OilTable[indexOilTable].ProcessIncomingOil = 0.0;
          } else {
            _OilTable[indexOilTable].OilStatus = OIL_IS_IDLE;
          }
        }
      }
      // when master is connected to other devices as a destination device
      else if (payloadUart[0] == ID_MASTER) {
        if (payloadUart[1] == ID_DEV_LMS_1) {
          enParsingData = true;
          indexOilTable = 0;
        } else if (payloadUart[1] == ID_DEV_LMS_2) {
          enParsingData = true;
          indexOilTable = 1;
        } else if (payloadUart[1] == ID_DEV_LMS_3) {
          enParsingData = true;
          indexOilTable = 2;
        } else if (payloadUart[1] == ID_DEV_LMS_4) {
          enParsingData = true;
          indexOilTable = 3;
        } else if (payloadUart[1] == ID_DEV_LMS_5) {
          enParsingData = true;
          indexOilTable = 4;
        } else if (payloadUart[1] == ID_DEV_LMS_6) {
          enParsingData = true;
          indexOilTable = 5;
        } else {
          enParsingData = false;
        }

        if (enParsingData) {
          // parsing data depend on type of PID Data
          if (payloadUart[2] == PID_DATA_STREAMING_OUTGOING) {
            _OilTable[indexOilTable].OilStatus = OIL_ON_OUTGOING_PROCESS;

            nextIndex = 3;

            // unused data
            // ================================================================================
            lengthInfoData =
                checkHeaderValueMiddleData(payloadUart, &nextIndex);
            for (uint8_t xx3 = 0; xx3 < lengthInfoData; xx3++) {
              unusedDataByte =
                  checkHeaderValueMiddleData(payloadUart, &nextIndex);
            }
            //============================================================================================

            // get current oil volume flowing through pipe
            _OilTable[indexOilTable].ProcessOutgoingOil =
                (double)(((checkHeaderValueMiddleData(payloadUart, &nextIndex)
                           << 16) |
                          (checkHeaderValueMiddleData(payloadUart, &nextIndex)
                           << 8) |
                          (checkHeaderValueMiddleData(payloadUart,
                                                      &nextIndex))) &
                         0xFFFFFF) /
                1000.0;
          } else if (payloadUart[2] == PID_DATA_STREAMING_INCOMING) {
            _OilTable[indexOilTable].OilStatus = OIL_ON_INCOMING_PROCESS;

            nextIndex = 11;

            // get current oil volume flowing through pipe
            _OilTable[indexOilTable].ProcessIncomingOil =
                (double)(((checkHeaderValueMiddleData(payloadUart, &nextIndex)
                           << 24) |
                          (checkHeaderValueMiddleData(payloadUart, &nextIndex)
                           << 16) |
                          (checkHeaderValueMiddleData(payloadUart, &nextIndex)
                           << 8) |
                          (checkHeaderValueMiddleData(payloadUart,
                                                      &nextIndex))) &
                         0xFFFFFF) /
                1000.0;

          } else if ((payloadUart[2] == PID_DATA_TRANSACTION_FULL_OUTGOING) ||
                     (payloadUart[2] ==
                      PID_DATA_TRANSACTION_TRIGGER_OUTGOING)) {
            nextIndex = 11;

            // unused data =========== OPERATOR NAME
            // =====================================================================
            lengthInfoData =
                checkHeaderValueMiddleData(payloadUart, &nextIndex);
            for (uint8_t xx3 = 0; xx3 < lengthInfoData; xx3++) {
              unusedDataByte =
                  checkHeaderValueMiddleData(payloadUart, &nextIndex);
            }
            //============================================================================================

            // unused data =========== UNIT ID ON SERVICE
            // =====================================================================
            lengthInfoData =
                checkHeaderValueMiddleData(payloadUart, &nextIndex);
            for (uint8_t xx3 = 0; xx3 < lengthInfoData; xx3++) {
              unusedDataByte =
                  checkHeaderValueMiddleData(payloadUart, &nextIndex);
            }
            //============================================================================================

            // unused data =========== UNIT MODEL
            // =====================================================================
            lengthInfoData =
                checkHeaderValueMiddleData(payloadUart, &nextIndex);
            for (uint8_t xx3 = 0; xx3 < lengthInfoData; xx3++) {
              unusedDataByte =
                  checkHeaderValueMiddleData(payloadUart, &nextIndex);
            }
            //============================================================================================

            // unused data =========== HM VALUE OF SERVICED UNIT
            // =====================================================================
            unusedData32 =
                (((checkHeaderValueMiddleData(payloadUart, &nextIndex) << 24) |
                  (checkHeaderValueMiddleData(payloadUart, &nextIndex) << 16) |
                  (checkHeaderValueMiddleData(payloadUart, &nextIndex) << 8) |
                  (checkHeaderValueMiddleData(payloadUart, &nextIndex))) &
                 0xFFFFFF);
            //============================================================================================

            // unused data =========== OIL TYPE
            // =====================================================================
            lengthInfoData =
                checkHeaderValueMiddleData(payloadUart, &nextIndex);
            for (uint8_t xx3 = 0; xx3 < lengthInfoData; xx3++) {
              unusedDataByte =
                  checkHeaderValueMiddleData(payloadUart, &nextIndex);
            }
            //============================================================================================

            // unused data =========== FILLING PROCESS TYPE
            // =====================================================================
            unusedDataByte =
                checkHeaderValueMiddleData(payloadUart, &nextIndex);
            //============================================================================================

            // unused data =========== ACTIVITY INFORMATION
            // =====================================================================
            unusedDataByte =
                checkHeaderValueMiddleData(payloadUart, &nextIndex);
            //============================================================================================

            // unused data =========== VALUE TARGET OF REFILLING OIL
            // =====================================================================
            unusedData32 =
                (((checkHeaderValueMiddleData(payloadUart, &nextIndex) << 16) |
                  (checkHeaderValueMiddleData(payloadUart, &nextIndex) << 8) |
                  (checkHeaderValueMiddleData(payloadUart, &nextIndex))) &
                 0xFFFFFF);
            //============================================================================================

            // get current oil volume flowing through pipe
            _OilTable[indexOilTable].ProcessOutgoingOil =
                (double)(((checkHeaderValueMiddleData(payloadUart, &nextIndex)
                           << 16) |
                          (checkHeaderValueMiddleData(payloadUart, &nextIndex)
                           << 8) |
                          (checkHeaderValueMiddleData(payloadUart,
                                                      &nextIndex))) &
                         0xFFFFFF) /
                1000.0;

            // get timestamp
            _OilTable[indexOilTable].totalisatorOil =
                (double)(((checkHeaderValueMiddleData(payloadUart, &nextIndex)
                           << 24) |
                          (checkHeaderValueMiddleData(payloadUart, &nextIndex)
                           << 16) |
                          (checkHeaderValueMiddleData(payloadUart, &nextIndex)
                           << 8) |
                          (checkHeaderValueMiddleData(payloadUart,
                                                      &nextIndex))) &
                         0xFFFFFF) /
                1000.0;
            _OilTable[indexOilTable].OilStatus = OIL_IS_IDLE;
          } else if ((payloadUart[2] == PID_DATA_TRANSACTION_INCOMING)) {
            nextIndex = 3;

            // unused data =========== OIL TYPE
            // =====================================================================
            lengthInfoData =
                checkHeaderValueMiddleData(payloadUart, &nextIndex);
            for (uint8_t xx3 = 0; xx3 < lengthInfoData; xx3++) {
              unusedDataByte =
                  checkHeaderValueMiddleData(payloadUart, &nextIndex);
            }
            //============================================================================================

            // unused data =========== OPERATOR NAME
            // =====================================================================
            lengthInfoData =
                checkHeaderValueMiddleData(payloadUart, &nextIndex);
            for (uint8_t xx3 = 0; xx3 < lengthInfoData; xx3++) {
              unusedDataByte =
                  checkHeaderValueMiddleData(payloadUart, &nextIndex);
            }
            //============================================================================================

            // unused data =========== UNIX START TIME
            // =====================================================================
            unusedData32 =
                (((checkHeaderValueMiddleData(payloadUart, &nextIndex) << 24) |
                  (checkHeaderValueMiddleData(payloadUart, &nextIndex) << 16) |
                  (checkHeaderValueMiddleData(payloadUart, &nextIndex) << 8) |
                  (checkHeaderValueMiddleData(payloadUart, &nextIndex))) &
                 0xFFFFFF);
            //============================================================================================

            // unused data =========== UNIX STOP TIME
            // =====================================================================
            unusedData32 =
                (((checkHeaderValueMiddleData(payloadUart, &nextIndex) << 24) |
                  (checkHeaderValueMiddleData(payloadUart, &nextIndex) << 16) |
                  (checkHeaderValueMiddleData(payloadUart, &nextIndex) << 8) |
                  (checkHeaderValueMiddleData(payloadUart, &nextIndex))) &
                 0xFFFFFF);
            //============================================================================================

            // get latest incoming oil flowing
            _OilTable[indexOilTable].ProcessIncomingOil =
                (double)(((checkHeaderValueMiddleData(payloadUart, &nextIndex)
                           << 24) |
                          (checkHeaderValueMiddleData(payloadUart, &nextIndex)
                           << 16) |
                          (checkHeaderValueMiddleData(payloadUart, &nextIndex)
                           << 8) |
                          (checkHeaderValueMiddleData(payloadUart,
                                                      &nextIndex))) &
                         0xFFFFFF) /
                1000.0;

            // get latest totalisator
            _OilTable[indexOilTable].totalisatorOil =
                (double)(((checkHeaderValueMiddleData(payloadUart, &nextIndex)
                           << 24) |
                          (checkHeaderValueMiddleData(payloadUart, &nextIndex)
                           << 16) |
                          (checkHeaderValueMiddleData(payloadUart, &nextIndex)
                           << 8) |
                          (checkHeaderValueMiddleData(payloadUart,
                                                      &nextIndex))) &
                         0xFFFFFF) /
                1000.0;

            _OilTable[indexOilTable].OilStatus = OIL_IS_IDLE;
          } else {
            _OilTable[indexOilTable].OilStatus = OIL_IS_IDLE;
          }
        }

      } else {
      }
    }
  }
}

/**
 * @brief function to get oil status either it's on idle or Incoming oil
 * processing or even outgoing oil processing
 * @return
 */
uint8_t ProtocolAA55::getStatusOil(TypeOfOil _oilType) {
  return (uint8_t)_OilTable[_oilType].OilStatus;
}

/**
 * @brief function to get data of transaction date on specific oil type
 * @return
 */
void ProtocolAA55::getDateTransaction(TypeOfOil _oilType, char *dateInfo_) {
  char bufftemp[11];
  sprintf(bufftemp, "%02d/%02d/%d", day(_OilTable[_oilType].timestamp),
          month(_OilTable[_oilType].timestamp),
          year(_OilTable[_oilType].timestamp));
  memcpy(dateInfo_, bufftemp, sizeof(bufftemp));
}

/**
 * @brief function to get data of transaction time on specific oil type
 * @return
 */
void ProtocolAA55::getTimeTransaction(TypeOfOil _oilType, char *timeInfo_) {
  char bufftemp[11];
  sprintf(bufftemp, "%02d:%02d:%02d", hour(_OilTable[_oilType].timestamp),
          minute(_OilTable[_oilType].timestamp),
          second(_OilTable[_oilType].timestamp));
  memcpy(timeInfo_, bufftemp, sizeof(bufftemp));
}

/**
 * @brief function to get unit ID which is on service to do change it's oil
 * engine
 * @return
 */
void ProtocolAA55::getIDUnitOnService(TypeOfOil _oilType, char *IDunit_) {
  memcpy(IDunit_, _OilTable[_oilType].unitIDonService,
         sizeof(_OilTable[_oilType].unitIDonService));
}

/**
 * @brief function to get Hour meter value of unit which on service to do change
 * it's oil engine
 * @return
 */
void ProtocolAA55::getValueofUnitHM(TypeOfOil _oilType, uint32_t *HMValue_) {
  (*HMValue_) = _OilTable[_oilType].HMValueUnit;
}

/**
 * @brief function to get operator name
 * @return
 */
void ProtocolAA55::getOperatorName(TypeOfOil _oilType, char *operatorName_) {
  memcpy(operatorName_, _OilTable[_oilType].operatorName,
         sizeof(_OilTable[_oilType].operatorName));
}

/**
 * @brief function to get oil name in the transaction
 * @return
 */
void ProtocolAA55::getOilName(TypeOfOil _oilType, char *oilNameSpecific_) {
  memcpy(oilNameSpecific_, _OilTable[_oilType].oilNameSpecific,
         sizeof(_OilTable[_oilType].oilNameSpecific));
}

/**
 * @brief function to get oil totalisator
 * @return
 */
void ProtocolAA55::getTotalisator(TypeOfOil _oilType, double *totalisatorOil) {
  (*totalisatorOil) = _OilTable[_oilType].totalisatorOil;
}

/**
 * @brief function to get current volume of incoming oil value into storage
 * @return
 */
void ProtocolAA55::getIncomingOilValue(TypeOfOil _oilType,
                                       double *OilIncomingValue) {
  (*OilIncomingValue) = _OilTable[_oilType].ProcessIncomingOil;
}

/**
 * @brief function to get current volume of outgoing oil value to unit engine
 * @return
 */
void ProtocolAA55::getOutgoingValue(TypeOfOil _oilType,
                                    double *OilOutgoingValue) {
  (*OilOutgoingValue) = _OilTable[_oilType].ProcessOutgoingOil;
}