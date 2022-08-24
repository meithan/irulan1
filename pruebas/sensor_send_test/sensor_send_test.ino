// This tests reading the LSM9DS1 sensor data and sending it out as LoRA messages

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // Required for LSM9DS1
#include "LoRaWan_APP.h"
#include "Arduino.h"

// -------------------------------------------
// LORA CONFIGURATION
#ifndef LoraWan_RGB
#define LoraWan_RGB 0
#endif

#define MHZ 1000000

#define RF_FREQUENCY                                915*MHZ   // RF frequency, in Hz
#define TX_OUTPUT_POWER                             20        // Output power, max 22 dBm
#define LORA_BANDWIDTH                              0         // Bandwidth, 0=125 kHz, 1=250 kHz, 2=500 kHz, 3=reserved
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // 1=4/6, 2=4/7, 3=4/7, 4=4/8
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
#define RX_TIMEOUT_VALUE                            1000      // milliseconds

// Payload and TX settings
#define PACKET_SIZE        64     // bytes
#define SEND_DELAY         1000    // milliseconds between sends

#define LED_GREEN 0x00ff00
#define LED_OFF 0

char packet[PACKET_SIZE];
static RadioEvents_t RadioEvents;
int16_t rssi,rxSize;
int count;

// -------------------------------------------
// LSM9DS1 CONFIGURATION

// I2C
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

// SPI
//#define LSM9DS1_SCK A5
//#define LSM9DS1_MISO 12
//#define LSM9DS1_MOSI A4
//#define LSM9DS1_XGCS 6
//#define LSM9DS1_MCS 5
// Software SPI
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_SCK, LSM9DS1_MISO, LSM9DS1_MOSI, LSM9DS1_XGCS, LSM9DS1_MCS);
// Hardware SPI! In this case, only CS pins are passed in
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);

void setup_LSM9DS1()
{
  // 1.) Set the accelerometer range
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}

float ax, ay, az, gx, gy, gz, mx, my, mz;
float values[9];

const int debug = true;

// -------------------------------------------

void setup() {
  
  Serial.begin(115200);

  count = 0;
  rssi = 0;

  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }

  // -----------------------
  // LSM9DS1 init 
  
  bool lsm_found;
  while (1) {
    lsm_found = lsm.begin();
    if (lsm_found) {
      break;
    } else {
      Serial.println("Unable to initialize the LSM9DS1!");
      delay(1000);
    }
  }
  Serial.println("Found LSM9DS1 9DOF IMU sensor");
  setup_LSM9DS1();      


  // -----------------------
  // LoRA init

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
    true, 0, 0, LORA_IQ_INVERSION_ON, 3000
  );  

}

void loop() {

  turnOnRGB(0x00ff00,0);

  // Read LSM9DS1
  lsm.read();
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);
  values[0] = a.acceleration.x; values[1] = a.acceleration.y; values[2] = a.acceleration.z;
  values[3] = g.gyro.x; values[4] = g.gyro.y; values[5] = g.gyro.z;
  values[6] = m.magnetic.x; values[7] = m.magnetic.x; values[8] = m.magnetic.x;

  if (debug) {
    Serial.println("-----------------------------------");
    Serial.print("ax = "); Serial.print(a.acceleration.x);
    Serial.print(", ay = "); Serial.print(a.acceleration.y);
    Serial.print(", az = "); Serial.println(g.gyro.z);
    Serial.print("gx = "); Serial.print(g.gyro.x);
    Serial.print(", gy = "); Serial.print(g.gyro.y);
    Serial.print(", gz = "); Serial.println(a.acceleration.z);
    Serial.print("mx = "); Serial.print(m.magnetic.x);
    Serial.print(", my = "); Serial.print(m.magnetic.y);
    Serial.print(", mz = "); Serial.println(m.magnetic.z);
  }

  // Assemble packet
  int i;
  char buf[10];
  packet[0] = 0;
  for (i = 0; i < 9; i++) {
    dtostrf(values[i], 6, 2, buf);
    strcat(packet, buf);
  }
  
  if (debug) {
    Serial.print("Packet payload: ");
    Serial.print(packet);
    Serial.print(", size=");
    Serial.print(strlen(packet));
    Serial.println();
  }

  // Transmit packet
  Radio.Send((uint8_t *)packet, strlen(packet));

  if (debug) {
    Serial.println("Sent packet");
  }

  turnOnRGB(0,0);
  delay(SEND_DELAY);

}
