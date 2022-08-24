//#include  <Wire.h>
//#include <SPI.h>
//#include "SPI.h"  
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!

// -------------------------------------------

// -------------------------------------------
// LSM9DS1 CONFIG

// I2C
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();


// SPI
#define LSM9DS1_SCK SCK
#define LSM9DS1_MISO MISO
#define LSM9DS1_MOSI MOSI
#define LSM9DS1_XGCS GPIO6
#define LSM9DS1_MCS GPIO7
// Software SPI
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_SCK, LSM9DS1_MISO, LSM9DS1_MOSI, LSM9DS1_XGCS, LSM9DS1_MCS);
// Hardware SPI: In this case, only CS pins are passed in
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);

void setupSensor()
{
  // 1.) Set the accelerometer range
//  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
//  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}

// Select which sensor to show (since showing all is too messsy)
const int SENSOR_ACCELEROMETER = 1;
const int SENSOR_GYROSCOPE     = 2;
const int SENSOR_MAGNETOMETER  = 3;
int show_sensor = SENSOR_ACCELEROMETER;

// -------------------------------------------

void setup() {
  Serial.begin(115200);

  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }

  Serial.println("LSM9DS1 data read demo");
    
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");

  // helper to just set the default scaling we want, see above!
  setupSensor();

  if (show_sensor == SENSOR_ACCELEROMETER) {
    Serial.println("ax ay az");
  } else if (show_sensor == SENSOR_GYROSCOPE) {
    Serial.println("gx gy gz");
  } else if (show_sensor == SENSOR_MAGNETOMETER) {
    Serial.println("mx my mz");
  }
  
}

void loop() 
{
  lsm.read();  /* ask it to read in the data */ 

  /* Get a new sensor event */ 
  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp); 

  Serial.println();

  Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
  Serial.print("\tY: "); Serial.print(a.acceleration.y);     Serial.print(" m/s^2 ");
  Serial.print("\tZ: "); Serial.print(a.acceleration.z);     Serial.println(" m/s^2 ");

  Serial.print("Mag X: "); Serial.print(m.magnetic.x);   Serial.print(" uT");
  Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" uT");
  Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.println(" uT");

  Serial.print("Gyro X: "); Serial.print(g.gyro.x);   Serial.print(" rad/s");
  Serial.print("\tY: "); Serial.print(g.gyro.y);      Serial.print(" rad/s");
  Serial.print("\tZ: "); Serial.print(g.gyro.z);      Serial.println(" rad/s");

/*
  if (show_sensor == SENSOR_ACCELEROMETER) {
    Serial.print(a.acceleration.x);
    Serial.print(" ");
    Serial.print(a.acceleration.y);
    Serial.print(" ");
    Serial.print(a.acceleration.z);
  } else if (show_sensor == SENSOR_GYROSCOPE) {
    Serial.print(g.gyro.x);
    Serial.print(" ");
    Serial.print(g.gyro.y);
    Serial.print(" ");
    Serial.print(g.gyro.z);    
  } else if (show_sensor == SENSOR_MAGNETOMETER) {
    Serial.print(m.magnetic.x);
    Serial.print(" ");
    Serial.print(m.magnetic.y);
    Serial.print(" ");
    Serial.print(m.magnetic.z);      
  }
  Serial.println();
*/

  delay(1000);

}
