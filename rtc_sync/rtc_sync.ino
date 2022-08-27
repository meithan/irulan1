/* Programa para sincronizar el reloj RTC (DS1307)
 *  
 * Este programa reinicia el RTC DS1307 sincronizando la hora
 * a la última hora de compilación del código, más un offset
 * configurable (aprox. el tiempo que toma al IDE compilar y 
 * subir el código al MCU).
 * 
 * El DS1307 se asume conectado por hardware I2C.
 * 
 * Desarrollado por: Sardaukar Rocketry Society, Agosto 2022
 */
#include "RTClib.h"
#include <time.h>

RTC_DS1307 rtc;

// ============================================================

// Fijar aquí el offset del reloj, que es aproximadamente
// el tiempo que toma compilar y subir el código al MCU
// TimeSpan(días, horas, minutos, segundos)
TimeSpan mcu_offset = TimeSpan(0,0,0,10);

// ============================================================

time_t t;
char *str;

void setup () {
  
  Serial.begin(115200);
  while (!Serial); // wait for serial port to connect. Needed for native USB
  delay(1000);

  if (!rtc.begin()) {
    Serial.println("No se pudo encontrar el RTC!");
    Serial.flush();
    while(1) delay(10);
  }

  delay(3000); Serial.println("----------------------------");

  DateTime set_time = DateTime(F(__DATE__), F(__TIME__)) + mcu_offset;

  Serial.print("Ajustando hora del RTC a ");
  t = set_time.unixtime();
  str = ctime(&t);
  Serial.println(str);
  rtc.adjust(set_time);

}

// ============================================================

void loop () {
    
    DateTime now = rtc.now();

    t = now.unixtime();
    str = ctime(&t);
    Serial.println(str);
    Serial.print(" (UNIX = ");
    Serial.print(now.unixtime());
    Serial.print(")");

    Serial.println();
    delay(1000);

}
