#ifndef ProtocolAA55_h
#define ProtocolAA55_h

#include <TimeLib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "Arduino.h"
#include "configProtocolAA55.h"

typedef enum {
  REFERENCE_DESTINATION_ID = 1,
  REFERENCE_SOURCE_ID      = 2,
  REFERENCE_PID_DATA       = 4,
} ListeningReference;

typedef enum {
  SAE_15W_40 = 0,
  SAE_30,
  SAE_10,
  SAE_60,
  TELLUS_86,
  COOLANT,
} TypeOfOil;

typedef enum {
  OIL_IS_IDLE = 0,
  OIL_ON_INCOMING_PROCESS,
  OIL_ON_OUTGOING_PROCESS,
} StatusOil;

typedef enum {
  ID_MASTER    = 0xA0,
  ID_DEV_LMS_1 = 0x01,
  ID_DEV_LMS_2 = 0x02,
  ID_DEV_LMS_3 = 0x03,
  ID_DEV_LMS_4 = 0x04,
  ID_DEV_LMS_5 = 0x05,
  ID_DEV_LMS_6 = 0x06,
} ListIDToListen;

typedef enum {
  PID_DATA_REQ_OIL_OUTGOING             = 2,
  PID_DATA_STREAMING_OUTGOING           = 4,
  PID_DATA_TRANSACTION_FULL_OUTGOING    = 5,
  PID_DATA_TRANSACTION_TRIGGER_OUTGOING = 6,
  PID_DATA_REQ_OIL_INCOMING             = 12,
  PID_DATA_STREAMING_INCOMING           = 9,
  PID_DATA_TRANSACTION_INCOMING         = 10,
} PIDDatatoListen;

typedef enum {
  DATA_IDLE = 0,
  DATA_COMPLETE,
  DATA_IS_PROCESSING,
} StatusReceivingData;

typedef struct {
  StatusOil OilStatus;
  uint32_t timestamp;
  char unitIDonService[6];
  uint32_t HMValueUnit;
  char operatorName[20];
  char oilNameSpecific[20];
  double totalisatorOil;
  double ProcessIncomingOil;
  double ProcessOutgoingOil;
} OilTableProperties;

class ProtocolAA55 {
 public:
  // for creating an objec
  ProtocolAA55(HardwareSerial *serial, uint8_t MyOwnIDDev);

  // for initialization
  bool begin(unsigned long baud, uint32_t config = SERIAL_8N1,
             int8_t rxPin = -1, int8_t txPin = -1, bool invert = false);

  void configListeningReference(uint8_t _ListeningReference);
  void configDirPinForRS485(int8_t pinDir = -1);

  void getBusData();
  void sendDatatoBusLine(uint8_t destinationID, uint8_t *packet,
                         uint8_t *lenData);

  uint8_t getStatusOil(TypeOfOil _oilType);
  void getDateTransaction(TypeOfOil _oilType, char *dateInfo_);
  void getTimeTransaction(TypeOfOil _oilType, char *timeInfo_);
  void getIDUnitOnService(TypeOfOil _oilType, char *IDunit_);
  void getValueofUnitHM(TypeOfOil _oilType, uint32_t *HMValue_);
  void getOperatorName(TypeOfOil _oilType, char *operatorName_);
  void getOilName(TypeOfOil _oilType, char *oilNameSpecific_);
  void getTotalisator(TypeOfOil _oilType, double *totalisatorOil);
  void getIncomingOilValue(TypeOfOil _oilType, double *OilIncomingValue);
  void getOutgoingValue(TypeOfOil _oilType, double *OilOutgoingValue);

 private:
  HardwareSerial *_hardwareSerial;
  OilTableProperties _OilTable[MAX_TYPE_OF_OIL];
  uint8_t _listeningRef;
  uint8_t _myID;
  uint8_t _dirPin;
  bool _listeningModeOnly = false;

  // hardware Serial variables to receive data
  uint8_t payloadUart[MAX_BUFFER_DATA];
  uint8_t UART_DataLength;
  bool CollectData          = false;
  bool FoundFirstTailData   = false;
  bool CheckDestinationData = false;
  bool HeaderBypass         = false;
  bool checkNextHeader      = false;
  bool SerialBusLineIsIdle  = false;
  bool checkPIDData         = false;
  bool checkSourceData      = false;

  uint8_t ListeningData();
  void SendPayloadUart(uint8_t *buf, uint8_t size_, uint16_t delayMicrosec);
  uint8_t checkHeaderValueMiddleData(uint8_t *buffPacket, uint8_t *idx);
  uint8_t addIdenfierWhenHeaderIsData(uint8_t currValue, uint8_t *buff,
                                      uint8_t *idx);
  void checksum_calc(uint8_t offsetByte, uint8_t *buf, uint8_t *size_);
  bool checksum_check(uint8_t startByte, uint8_t *buf, uint8_t *size_);
};

#endif