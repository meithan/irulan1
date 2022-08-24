// This tests I/O on the SD card reader via SPI

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "LoRaWan_APP.h"   // for RGB
//#include "Arduino.h"

// -------------------------------------------
// HTCC-AB01 GENERAL CONFIGURATION
#ifndef LoraWan_RGB
#define LoraWan_RGB 0
#endif

#define LED_GREEN 0x00ff00
#define LED_OFF 0

// -------------------------------------------
// SD card reader configuration

// SPI
#define SPI_CS GPIO0

// -------------------------------------------

const int debug = true;
bool status;
File myFile;

// -------------------------------------------

void setup() {
  
  Serial.begin(115200);

  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }

  // ----------------------
  // SPI initialization
  
  status = SPI.begin(SCK,MISO,MOSI,SPI_CS);
  SPI.setFrequency(1000000);
  pinMode(SPI_CS,OUTPUT);
  if (debug) {
    Serial.print("SPI init = "); Serial.println(status);
    Serial.print("SPI SS/CS pin = "); Serial.println(SPI_CS);
  }
  SPI.end();

  // ----------------------
  // Initialize SD reader
  
  Serial.print("Initializing SD card reader ...");
  if (!SD.begin(SPI_CS)) {
    Serial.println(" failed!");
//    turnOnRGB(0xff0000,0);
    while(1);
  }
  Serial.println(" done.");

  turnOnRGB(0x00ff00,0);

}

void loop() {

//  turnOnRGB(0x00ff00,0);

  delay(1000);

}
