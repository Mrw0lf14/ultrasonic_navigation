// This is a simple example to test communication with the motors
// It should make all motor led blink once per second
// Default baudrate is 1000000

#include "DxlMaster2.h"

const long unsigned int baudrate = 57600;

// Use broadcast address to affect all connected motors
DynamixelDevice broadcast_device(BROADCAST_ID);

uint8_t led_state = true;

void setup() {
  DxlMaster.begin(baudrate);
  broadcast_device.protocolVersion(2);
  delay(100);
}

void loop() {
  broadcast_device.write(DYN2_ADDR_LED, led_state);
  led_state = !led_state;
  delay(1000);
}
