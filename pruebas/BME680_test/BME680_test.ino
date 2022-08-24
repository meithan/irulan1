/***************************************************************************
  This is a library for the BME680 gas, humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME680 Breakout
  ----> http://www.adafruit.com/products/3660

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

// For SPI
//#define BME_SCK 27
//#define BME_MOSI 25
//#define BME_MISO 26
#define BME_CS GPIO11

#define QNH_INHG (30.37)

#define INHG_to_HPA (33.86389)
#define SEALEVELPRESSURE_HPA (QNH_INHG*INHG_to_HPA)
//#define SEALEVELPRESSURE_HPA (1013.25)

// Select communication protocol
//Adafruit_BME680 bme; // I2C
Adafruit_BME680 bme(BME_CS); // hardware SPI
//Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);

bool use_plotter = false;

void setup() {
  
  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("BME680 test"));

  if (!bme.begin(0x77)) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  } else {
    if (!use_plotter) Serial.println(F("BME680 sensor detected."));
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
//  bme.setGasHeater(320, 150); // 320*C for 150 ms
  bme.setGasHeater(0, 0); // turn off gas sensor

  if (use_plotter) Serial.println("Humidty Temperature Pressure Gas");
  
}

float altitude;

void loop() {
  
  if (!bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }

  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

  if (!use_plotter) {
     
  Serial.print("Temperature = ");
  Serial.print(bme.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(bme.humidity);
  Serial.println(" %");

  Serial.print("Gas = ");
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(" KOhms");

  Serial.print("Altitude = ");
  Serial.print(altitude);
  Serial.print(" m (QNH=");
  Serial.print(QNH_INHG);
  Serial.println(")");

  Serial.println();

  } else {

  Serial.print(bme.humidity); Serial.print(" ");
  Serial.print(bme.temperature); Serial.print(" ");
  Serial.print(bme.pressure / 100.0); Serial.print(" ");
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println();

  }
  
  delay(1000);

}
