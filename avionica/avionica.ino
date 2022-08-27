/* Programa principal de aviónica para el Irulan-1
 * v0.9
 *  
 * La aviónica lee los valores de múltiples sensores y los transmite por radio.
 * El CPU es un ASR6502 a bordo de una placa de desarrollo Heltec HTCC-AB02S.
 * 
 * Los sensores leídos son:
 *   - Latitud, longitud, altitud y velocidad obtenidos del GPS a bordo (AIR530Z).
 *   - Presión barométrica, temperatura y humedad de un sensor atmosférico BME680.
 *   - Aceleración lineal, velocidad angular y campo magnético triaxiales de un IMU LSM9DS1.
 *   - Nivel de voltaje de la batería LiPo principal
 *   - Hora según un RTC1307 (batería CR2302 independiente)
 *   
 * Estos valores son empaquetados y transmitidos por radio LoRA (SX1262) 915 Mhz a la estación 
 * terrena, marcando cada mensaje con el tiempo del RTC.
 * 
 * Desarrollado por: Sardaukar Rocketry Society, Agosto 2022
 */

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include "Zanshin_BME680.h"
#include "RTClib.h"
//#include "GPS_Air530.h"
#include "GPS_Air530Z.h"
#include "HT_SSD1306Wire.h"
#include "LoRaWan_APP.h"

// ===========================================================
// Configuración y constantes globales

const char* version_str = "v0.9";

const unsigned long cycle_time = 1000;   // Milisegundos

#define BUF_SIZE 12
const uint32_t SERIAL_SPEED = 115200;
const bool debug = true;

// ===========================================================
// Variables globales

// RTC
char rtc_time_str[19];

//LSM9DS1
float ax, ay, az, gx, gy, gz, mx, my, mz;

// BME680
const float QNH_INHG = 30.32;
float bme_pres, bme_humid, bme_temp, baro_alt, baro_alt_start, baro_height;
static int32_t _temp, _humid, _pres, _gas;
bool bme_success;

uint16_t bat_voltage;
uint8_t bat_level;

char line_buf[18];
char buf1[BUF_SIZE], buf2[BUF_SIZE], buf3[BUF_SIZE];

unsigned long next_millis = 0;

// ===========================================================
// Funciones de inicialización y configuración de hardware de los sensores

// -------------------------------------
// LSM9DS1

// I2C
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
sensors_event_t accel, magn, gyro, imu_temp;

void setup_LSM9DS1() {

  // Intentamos inicializar el LSM9DS1
  if (!lsm.begin()) {
    if (debug) Serial.println("No se pudo detectar el LSM9DS1! :(");
    while (1);
  }
  if (debug) Serial.println("LSM9DS1 inicializado");
 
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

// -------------------------------------
// BME680

const uint8_t BME_SPI_CS_PIN = GPIO11;
BME680_Class BME680;

const float INHG_to_MB = 33.86389;
const float SEALEVELPRESSURE_MB = QNH_INHG * INHG_to_MB;

void setup_BME680() {
  
  while (!BME680.begin(BME_SPI_CS_PIN)) {
    if (debug) Serial.println("No se pudo iniciliazar BME680! :(");
    while (1);
  }
  if (debug) Serial.println("BME680 inicializado");
  
  BME680.setOversampling(TemperatureSensor, Oversample16);
  BME680.setOversampling(HumiditySensor, Oversample16);
  BME680.setOversampling(PressureSensor, Oversample16);
  BME680.setIIRFilter(IIR16);
  
  // Desactivamos el sensor de gas para tener lectura de temperatura más limpia
  BME680.setGas(0, 0);

  BME680.getSensorData(_temp, _humid, _pres, _gas);
  baro_alt_start = calc_altitude(_pres);

}

float calc_altitude(const int32_t press) {
  static float alt_result;
  alt_result = 44330.0 * (1.0 - pow(((float)press / 100.0) / SEALEVELPRESSURE_MB, 0.1903));
  return (alt_result);
}  

// -------------------------------------
// RTC1307 

RTC_DS1307 rtc;
DateTime now;
TimeSpan mcu_offset = TimeSpan(0,0,0,10);

void setup_RTC1307() {

  if (!rtc.begin()) {
    if (debug) {
      Serial.println("No se pudo inicializar RTC1307!");
      Serial.flush();
    }
    while (1) delay(10);
  }
  if (debug) Serial.println("RTC DS1307 inicializado");

  if (!rtc.isrunning()) {
    if (debug) {
      Serial.println("RTC no está corriendo!");
      Serial.println("Cargar sketch para resetear el RTC");
    }
    while (1) delay(10);
  } else {
    if (debug) Serial.println("RTC is running");
  }

}

// -------------------------------------
// GPS

Air530ZClass GPS;

void setup_GPS() {
  GPS.begin();
  GPS.setmode(MODE_GPS_GLONASS);
}

// -------------------------------------
// Pantalla OLED

SSD1306Wire oled(0x3c, 500000, SDA, SCL, GEOMETRY_128_64, GPIO10); // addr , freq , SDA, SCL, resolution , rst

void setup_OLED() {
  
  oled.init();
  oled.clear();
  oled.display();

}

// -------------------------------------
// LoRa

#ifndef LoraWan_RGB
#define LoraWan_RGB 0
#endif

#define MHZ 1000000

#define RF_FREQUENCY                      915*MHZ   // RF frequency, in Hz
#define TX_OUTPUT_POWER                   22        // Output power, max 22 dBm
#define LORA_BANDWIDTH                    0         // Bandwidth, 0=125 kHz, 1=250 kHz, 2=500 kHz, 3=reserved
#define LORA_SPREADING_FACTOR             7         // [SF7..SF12]
#define LORA_CODINGRATE                   1         // 1=4/6, 2=4/7, 3=4/7, 4=4/8
#define LORA_PREAMBLE_LENGTH              8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT               0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON        false
#define LORA_IQ_INVERSION_ON              false

// Payload and TX settings
#define PACKET_SIZE        160     // bytes

#define LED_COLOR_RED 0xff0000
#define LED_COLOR_GREEN 0x00ff00
#define LED_OFF 0

char packet[PACKET_SIZE];
static RadioEvents_t RadioEvents;
int16_t rssi, rxSize;

void setup_LoRa() {
  
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
    true, 0, 0, LORA_IQ_INVERSION_ON, 3000
  );  

}

// ===========================================================
// Inicialización principal

void setup() {

  if (debug) {
    Serial.begin(SERIAL_SPEED);
  }

  // Esperamos 1 segundo para darle tiempo al Serial
  delay(1000);

  if (debug) {
    Serial.println();
    Serial.println("*************************");
    Serial.println("*   Avionica Irulan-1   *");
    Serial.print("*        "); Serial.print(version_str); Serial.println("           *");
    Serial.println("*************************");
  }

  setup_OLED();
  
  oled.setTextAlignment(TEXT_ALIGN_CENTER);
  oled.setFont(ArialMT_Plain_16);
  oled.drawString(64, 32-12, "Avionica Irulan-1");
  oled.drawString(64, 32+8, "Inicializando...");
  oled.display();
  
  delay(1000);

  if (debug) {
    Serial.println();
    Serial.println("Inicializando sensores ...");
  }
  oled.clear();
  oled.setTextAlignment(TEXT_ALIGN_LEFT);
  oled.setFont(ArialMT_Plain_16);

  setup_RTC1307();
  oled.drawString(0, 0, "DS1307 ... OK");
  oled.display();

  setup_LSM9DS1();
  oled.drawString(0, 16, "LSM9DS1 ... OK");
  oled.display();
  
  setup_BME680();
  oled.drawString(0, 32, "BME680 ... OK");
  oled.display();

  setup_GPS();
  oled.drawString(0, 48, "GPS ... OK");
  oled.display();

  setup_LoRa();

  // Todo listo
  delay(1000);
  oled.clear();
  oled.setTextAlignment(TEXT_ALIGN_LEFT);
  oled.setFont(ArialMT_Plain_10);  
 
}

// ===========================================================
// Bucle principal

void loop() {

  next_millis = millis() + cycle_time;

  if (debug) Serial.println();
  
  // --------------------------------------
  // RTC1307
  now = rtc.now();
  sprintf(rtc_time_str, "%02u-%02u-%02u %02u:%02u:%02u", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  
  if (debug) {
    Serial.print("RTC ");
    Serial.print(rtc_time_str);
    Serial.print(" (");
    Serial.print("UNIX = ");
    Serial.print(now.unixtime());
    Serial.println(")");
  }

  // --------------------------------------
  // LSM9DS1
  
  lsm.read();
  lsm.getEvent(&accel, &magn, &gyro, &imu_temp); 

  ax = accel.acceleration.x; ay = accel.acceleration.y; az = accel.acceleration.z;
  gx = gyro.gyro.x; gy = gyro.gyro.y; gz = gyro.gyro.z;
  mx = magn.magnetic.x; my = magn.magnetic.y; mz = magn.magnetic.z;

  if (debug) {
    dtostrf(ax, 0, 2, buf1); dtostrf(ay, 0, 2, buf2); dtostrf(az, 0, 2, buf3);
    Serial.printf("accel = (%s, %s, %s) m/s^2\n", buf1, buf2, buf3);
    dtostrf(gx, 0, 2, buf1); dtostrf(gy, 0, 2, buf2); dtostrf(gz, 0, 2, buf3);
    Serial.printf("gyro = (%s, %s, %s) rad/s\n", buf1, buf2, buf3);
    dtostrf(mx, 0, 2, buf1); dtostrf(my, 0, 2, buf2); dtostrf(mz, 0, 2, buf3);
    Serial.printf("magn = (%s, %s, %s) G\n", buf1, buf2, buf3);
  }
  
  // --------------------------------------
  // BME680
  
  BME680.getSensorData(_temp, _humid, _pres, _gas);
  bme_temp = ((float)_temp) / 100;
  bme_humid = ((float)_humid) / 1000;
  bme_pres = ((float)_pres) / 100;
  baro_alt = calc_altitude(_pres);
  baro_height = baro_alt - baro_alt_start;

  if (debug) {
    Serial.print(baro_alt); Serial.print(" m, ");
    Serial.print(baro_height); Serial.print(" m, ");
    Serial.print(bme_temp); Serial.print(" degC, ");
    Serial.print(bme_pres); Serial.print(" hPa, ");
    Serial.print(bme_humid); Serial.print(" RH%");
    Serial.println();
  }

  // -------------------------------------- 
  // GPS

  GPS.encode(GPS.read());
  
  if (debug) {  
    Serial.print("lat "); Serial.print(GPS.location.lat());
    Serial.print(", lon "); Serial.print(GPS.location.lng());
    Serial.print(", sats "); Serial.print(GPS.satellites.value());
    Serial.println();
  }

  // -------------------------------------- 
  // Voltage de la batería

//  bat_voltage = getBatteryVoltage();
  bat_level = BoardGetBatteryLevel();

  // --------------------------------------
  // LORA

  // Armar paquete
  packet[0] = '\0';
  
  // UNIX timestamp
  snprintf(buf1, BUF_SIZE, "%d", now.unixtime());
  strcat(packet, buf1);

  // GPS data
  dtostrf(GPS.location.lat(), 0, 4, buf1); strcat(packet, ","); strcat(packet, buf1);
  dtostrf(GPS.location.lng(), 0, 4, buf1); strcat(packet, ","); strcat(packet, buf1);
  snprintf(buf1, BUF_SIZE, "%d", (int)GPS.altitude.meters()); strcat(packet, ","); strcat(packet, buf1);
  dtostrf(GPS.speed.kmph(), 0, 1, buf1); strcat(packet, ","); strcat(packet, buf1);
  snprintf(buf1, BUF_SIZE, "%d", (int)GPS.satellites.value()); strcat(packet, ","); strcat(packet, buf1);

  // Atmo data
  dtostrf(baro_alt, 0, 1, buf1); strcat(packet, ","); strcat(packet, buf1);
  dtostrf(baro_height, 0, 1, buf1); strcat(packet, ","); strcat(packet, buf1);
  dtostrf(bme_temp, 0, 1, buf1); strcat(packet, ","); strcat(packet, buf1);
  dtostrf(bme_pres, 0, 1, buf1); strcat(packet, ","); strcat(packet, buf1);
  dtostrf(bme_humid, 0, 1, buf1); strcat(packet, ","); strcat(packet, buf1);

  // Acceloremeter
  dtostrf(ax, 0, 2, buf1); strcat(packet, ","); strcat(packet, buf1);
  dtostrf(ay, 0, 2, buf1); strcat(packet, ","); strcat(packet, buf1);
  dtostrf(az, 0, 2, buf1);; strcat(packet, ","); strcat(packet, buf1);

  // Gyroscope
  dtostrf(gx, 0, 2, buf1); strcat(packet, ","); strcat(packet, buf1);
  dtostrf(gy, 0, 2, buf1); strcat(packet, ","); strcat(packet, buf1);
  dtostrf(gz, 0, 2, buf1); strcat(packet, ","); strcat(packet, buf1);

  // Magnetometer
  dtostrf(mx, 0, 2, buf1); strcat(packet, ","); strcat(packet, buf1);
  dtostrf(my, 0, 2, buf1); strcat(packet, ","); strcat(packet, buf1);
  dtostrf(mz, 0, 2, buf1); strcat(packet, ","); strcat(packet, buf1);  

  // Voltaje y nivel de la batería
  dtostrf(bat_voltage, 0, 2, buf1); strcat(packet, ","); strcat(packet, buf1);
  snprintf(buf1, BUF_SIZE, "%d", (int)bat_level); strcat(packet, ","); strcat(packet, buf1); 

  // Transmitir paquete
  turnOnRGB(LED_COLOR_RED, 0);
  if (debug) {
    Serial.printf("Tamaño de paquete: %d bytes\n", strlen(packet));
    Serial.println(packet);
    Serial.println("Transmitiendo telemetría ...");
  }
  Radio.Send((uint8_t*)packet, strlen(packet));
  if (debug) Serial.println("TX completada");
  turnOffRGB();
  
  // --------------------------------------
  // Imprimir datos a OLED
  oled.clear();
  oled.drawString(0, 0, rtc_time_str);
  
  dtostrf(GPS.location.lat(), 0, 4, buf1);
  dtostrf(GPS.location.lng(), 0, 4, buf2);
  sprintf(line_buf, "lat %s lon %s %d", buf1, buf2, GPS.satellites.value());
  oled.drawString(0, 12, line_buf);

  dtostrf(baro_alt, 0, 1, buf1);
  dtostrf(baro_height, 0, 1, buf2);
  dtostrf(bme_temp, 0, 1, buf3);
  sprintf(line_buf, "alt %s [%s], %sC", buf1, buf2, buf3);
  oled.drawString(0, 24, line_buf);

  dtostrf(ax, 0, 2, buf1);
  dtostrf(ay, 0, 2, buf2);
  dtostrf(az, 0, 2, buf3);
  sprintf(line_buf, "accel (%s,%s,%s)", buf1, buf2, buf3);
  oled.drawString(0, 36, line_buf);

  dtostrf(gx, 0, 2, buf1);
  dtostrf(gy, 0, 2, buf2);
  dtostrf(gz, 0, 2, buf3);
  sprintf(line_buf, "gyro (%s,%s,%s)", buf1, buf2, buf3);
  oled.drawString(0, 48, line_buf);

//  sprintf(line_buf, "magn (%d.%02d,%d.%02d,%d.%02d)", (int)mx, (int)(abs(mx)*100)%100, (int)my, (int)(abs(my)*100)%100, (int)mz, (int)(abs(mz)*100)%100);
//  oled.drawString(0, 48, line_buf);
  
  oled.display();

  // --------------------------------------
  if (next_millis > millis()) delay(next_millis - millis());

}
