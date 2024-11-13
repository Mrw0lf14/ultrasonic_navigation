#include <DxlMaster2.h>

const unsigned long dynamixel_baudrate = 57600;
const unsigned long serial_baudrate = 57600;

DynamixelConsole console(Serial, 2);

void setup() {
  DxlMaster.begin(dynamixel_baudrate);
  Serial.begin(serial_baudrate);
}

void loop() {
  console.loop();
}
