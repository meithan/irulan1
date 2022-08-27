/* Heltec Automation Receive communication test example
 *
 * Function:
 * 1. Receive the same frequency band lora signal program
 * 
 * 
 * this project also realess in GitHub:
 * https://github.com/HelTecAutomation/ASR650x-Arduino
 * */

#include "LoRaWan_APP.h"
#include "Arduino.h"

// ===========================================================

const uint32_t SERIAL_SPEED = 115200;

const unsigned long cycle_time = 500;   // Milisegundos
const unsigned long led_on_time = 250;

const bool debug = true;

// ===========================================================

/*
 * set LoraWan_RGB to 1,the RGB active in loraWan
 * RGB red means sending;
 * RGB green means received done;
 */
#ifndef LoraWan_RGB
#define LoraWan_RGB 0
#endif

#define MHZ 1000000

#define RF_FREQUENCY                      915*MHZ   // RF frequency, in Hz
#define LORA_BANDWIDTH                    0         // Bandwidth, 0=125 kHz, 1=250 kHz, 2=500 kHz, 3=reserved
#define LORA_SPREADING_FACTOR             7         // [SF7..SF12]
#define LORA_CODINGRATE                   1         // 1=4/6, 2=4/7, 3=4/7, 4=4/8
#define LORA_PREAMBLE_LENGTH              8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT               0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON        false
#define LORA_IQ_INVERSION_ON              false

#define RX_TIMEOUT_VALUE                  1000
#define MAX_PACKET_SIZE                   128

#define LED_COLOR_RED 0xff0000
#define LED_COLOR_GREEN 0x00ff00
#define LED_OFF 0

char rxpacket[MAX_PACKET_SIZE];
static RadioEvents_t RadioEvents;
int16_t packetNumber;
int16_t rssi, rxSize, snr;

unsigned long next_cycle_time;
unsigned long next_led_off;
unsigned long last_rx;
int led_state;

char buf1[32];

// ===========================================================

// Convierte un float en una string decimal, con decs decimales
void f2s (char* buf, float value, unsigned int decs) {
  buf[0] = 0;
  float fractpart, intpart;
  fractpart = modf(value, &intpart);
  fractpart = fabs(fractpart) * (pow(10,decs));
  sprintf(buf, "%d.%d", (int)(intpart), (int)(fractpart));
}

void setup() {

  if (debug) Serial.begin(SERIAL_SPEED);

  if (debug) {
    delay(1000);
    Serial.println();
    Serial.println("==========================");
    Serial.println("Inicializando radio LoRa ...");
  }
   
  packetNumber = 0;
  rssi = 0;
  snr = 0;
	
  RadioEvents.RxDone = OnRxDone;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
	
	Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
     LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
     LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
     0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
  
  turnOnRGB(LED_COLOR_RED, 0);

  if (debug) {
    f2s(buf1, RF_FREQUENCY/MHZ, 3);
    Serial.printf("Escuchando en %s MHz\r\n", buf1);
  }

  next_cycle_time = 0;
  led_state = 0;
  last_rx = 0;
   
}

// ===========================================================

void loop() {
//
//  if ((led_state == 0) && (millis() - last_rx > 5000)) {
//    led_state = 2;
//    turnOnRGB(LED_COLOR_RED, 0);
//  }
//
  if (led_state == 1) {
    if (millis() >= next_led_off) {
      turnOffRGB();
      led_state = 0;
    }
  } else if (led_state == 2) {
    
  }

  if (millis() >= next_cycle_time) {

    next_cycle_time += cycle_time;
    
	  Radio.Rx(0);
//    Radio.IrqP/rocess();

  }

}

void OnRxDone(uint8_t *payload, uint16_t _size, int16_t _rssi, int8_t _snr) {

    packetNumber += 1;
    rssi = _rssi;
    snr = _snr;
    rxSize = _size;
    memcpy(rxpacket, payload, _size);
    rxpacket[_size] = '\0';
    
    turnOnRGB(LED_COLOR_GREEN, 0);
    led_state = 1;
    next_led_off = millis() + led_on_time;
    last_rx = millis();
    
    Radio.Sleep( );
    Serial.printf("\r\nPaquete #%d recibido, %d bytes, rssi=%d, snr=%d\r\n", packetNumber, rxSize, rssi, snr);
    Serial.println(rxpacket);
    
}
