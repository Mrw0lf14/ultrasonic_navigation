// Demonstrate how to change the communication baudrate of a motor
// This can be usefull if you want to reduce baudrate, in order to
// use a software serial, or if you need 2Mbaud for high speed communication


#include "DxlMaster2.h"



// motor id
const uint8_t id = 1;
const unsigned long baudrate_before = 57600;
const unsigned long baudrate_after = 1000000;

DynamixelMotor motor(id);

// the led should blink twice
void setup() 
{
  // start communication at old baudrate
  DxlMaster.begin(baudrate_before);
  motor.protocolVersion(2);
  delay(100);
  
  // make the motor led blink once
  motor.init();
  motor.led(HIGH);
  delay(1000);
  motor.led(LOW);
  delay(1000);
  
  // set motor baudrate to new baudrate
  motor.communicationSpeed(baudrate_after);
  
  // set arduino baudrate at new baudrate
  delay(100);
  DxlMaster.begin(baudrate_after);
  
  // make the motor led blink once
  motor.led(HIGH);
  delay(1000);
  motor.led(LOW);
  delay(1000);
  
}

void loop(){
  motor.led(HIGH);
  delay(1000);
  motor.led(LOW);
  delay(1000);
}
