/*
  Example of Protocol AA dan 55
  =====================================
*/
#include "ProtocolAA55.h"

#define DESTINATION_MASTER_ID			0xA0
#define BAUDRATE_RS485					115200
#define RX_PIN							18
#define TX_PIN							19

ProtocolAA55 protocol(&Serial1, 0xC0);

uint8_t samplePacketData[25] = 
{
  0x10, 0x11, 0xAA, 0xA5, 0x01,
  0x20, 0x11, 0xAA, 0xA5, 0x01,
  0x30, 0x11, 0xAA, 0x5A, 0x01,
  0x40, 0x11, 0x55, 0x5A, 0x01,
  0x50, 0x11, 0x55, 0x5A, 0x01,
};
  
void setup(){
  Serial.begin(115200);
  delay(1000); //Take some time to open up the Serial Monitor

  protocol.begin(BAUDRATE_RS485, SERIAL_8N1, RX_PIN, TX_PIN, false);
  intervalCounterPrev = millis();
}

void loop(){
  if((millis() - intervalCounterPrev) >= 1000)
  {
    intervalCounterPrev = millis();

    uint8_t lendData = sizeof(samplePacketData);
    protocol.sendDatatoBusLine(DESTINATION_MASTER_ID, samplePacketData, &lendData);
    Serial.printf("RS485 SENT... %d \r\n", counterToSleep);
  }
}
