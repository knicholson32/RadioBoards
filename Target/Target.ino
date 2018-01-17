// **********************************************************************************
// This sketch is an example of how wireless programming can be achieved with a Moteino
// that was loaded with a custom 1k bootloader (DualOptiboot) that is capable of loading
// a new sketch from an external SPI flash chip
// The sketch includes logic to receive the new sketch 'over-the-air' and store it in
// the FLASH chip, then restart the Moteino so the bootloader can continue the job of
// actually reflashing the internal flash memory from the external FLASH memory chip flash image
// The handshake protocol that receives the sketch wirelessly by means of the RFM69 radio
// is handled by the SPIFLash/RFM69_OTA library, which also relies on the RFM69 library
// These libraries and custom 1k Optiboot bootloader are at: http://github.com/lowpowerlab
// **********************************************************************************
// Copyright Felix Rusu 2016, http://www.LowPowerLab.com/contact
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it
// and/or modify it under the terms of the GNU General
// Public License as published by the Free Software
// Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will
// be useful, but WITHOUT ANY WARRANTY; without even the
// implied warranty of MERCHANTABILITY or FITNESS FOR A
// PARTICULAR PURPOSE. See the GNU General Public
// License for more details.
//
// Licence can be viewed at
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************
#include <RFM69.h>         //get it here: https://github.com/lowpowerlab/RFM69
#include <RFM69_ATC.h>     //get it here: https://github.com/lowpowerlab/RFM69
#include <RFM69_OTA.h>     //get it here: https://github.com/lowpowerlab/RFM69
#include <SPIFlash.h>      //get it here: https://github.com/lowpowerlab/spiflash
#include <SPI.h>           //included with Arduino IDE install (www.arduino.cc)
#include <EEPROM.h>

// Radio Settings
//#define NODEID      1       // node ID used for this unit
#define NETWORKID   100 // Network ID
#define FREQUENCY     RF69_915MHZ
#define FREQUENCY_EXACT 916000000 // Exact xmit freq
#define ATC_RSSI      -75 // Autopower RSSI
#define SERIAL_BAUD 115200 // Serial Baud Rate
#define ACK_TIME    30  // # of ms to wait for an ack
#define ENCRYPTKEY "sampleEncryptKey" //(16 bytes of your choice - keep the same on all encrypted nodes)
#define BLINKPERIOD 500

#define LED           9 // Moteinos hsave LEDs on D9
#define FLASH_SS      8 // and FLASH SS on D8

RFM69 radio;

char input = 0;
long lastPeriod = -1;

//*****************************************************************************************************************************
// flash(SPI_CS, MANUFACTURER_ID)
// SPI_CS          - CS pin attached to SPI flash chip (8 in case of Moteino)
// MANUFACTURER_ID - OPTIONAL, 0x1F44 for adesto(ex atmel) 4mbit flash
//                             0xEF30 for windbond 4mbit flash
//                             0xEF40 for windbond 16/64mbit flash
//*****************************************************************************************************************************
SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for windbond 4mbit flash

int pwmVal;
bool pwmDir;
int nodeId;
void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  Serial.begin(SERIAL_BAUD);

#ifdef NODEID
  EEPROM.write(0, (byte)NODEID);
  nodeId = NODEID;
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#else
  nodeId = (int)EEPROM.read(0);
  radio.initialize(FREQUENCY,nodeId,NETWORKID);
#endif

  
  radio.encrypt(ENCRYPTKEY); //OPTIONAL
  radio.setFrequency(FREQUENCY_EXACT); //set frequency to some custom frequency
  radio.setHighPower(); //must include this only for RFM69HW/HCW!

  Serial.print("Start node...");
  Serial.print("Node ID = ");
  Serial.println(nodeId);

  if (flash.initialize())
    Serial.println("SPI Flash Init OK!");
  else
    Serial.println("SPI Flash Init FAIL!");

  pwmVal = 0;
  pwmDir = true;

}

static char sendbuffer[62];

void loop(){
  // Check for recieved data from the radio
  if (radio.receiveDone()) {
    // MUST Check for hex data FIRST. This needs to run enough so that we dont miss a new upload
    CheckForWirelessHEX(radio, flash, false);

    if (radio.ACKRequested())
    {
      radio.sendACK();
      Serial.print(" - ACK sent");
    }

    delay(10);
    
//    Serial.print('[');Serial.print(radio.SENDERID, DEC);Serial.print("] ");
//    for (byte i = 0; i < radio.DATALEN; i++)
//      Serial.print((char)radio.DATA[i]);
//    Serial.print("   [RX_RSSI:");Serial.print(radio.RSSI);Serial.print("]");

    int len = radio.DATALEN;

    for(int i=0; i < len; i++)
      sendbuffer[i] = (char)radio.DATA[i];    
    
    Serial.println("Reply");
    radio.send(radio.SENDERID, sendbuffer, len, 1);

    
    
    Blink(LED,3);
    Serial.println();
    
    // Do whatever else with the data
  }
  //else Serial.print('.');
//  Blink(LED,10); //heartbeat
  delay(10);
}

  

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}
