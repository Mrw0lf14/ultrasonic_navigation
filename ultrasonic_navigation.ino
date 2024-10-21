#include <HardwareSerial.h>
#include <ESPAsyncWebServer.h>
#include "ARA_ESP.h"
#include <math.h>
float angle = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 2, 4);
  esp.begin(Serial2);  /* Запустить сервис по отправки данных в полетный контроллер */
  // esp.gps_num_sat(5);
  //                (37697128
  //                (370000928
  //                (4294966877)
  esp.gps_longitude(37.697118);
  esp.gps_latitude(55.733153);
  esp.gps_altitude(1500);
  esp.gps_local_position(0,0,0);
  esp.gps_set_lpos_orientation(200);
  // delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  esp.gps_local_position(0,0,1);
  delay(500);
  esp.gps_local_position(0,10,2);
  delay(500);
  esp.gps_local_position(40,10,3);
  delay(500);
  esp.gps_local_position(40,0,2);
  delay(500);
}
