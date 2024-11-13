// Test motor joint mode


#include "DxlMaster2.h"

// id of the motor
const uint8_t id = 1;
// speed, between 0 and 1023
int16_t speed = 512;
// communication baudrate
const long unsigned int baudrate = 57600;

DynamixelMotor motor(id);

void setup()
{ 
  DxlMaster.begin(baudrate);
  motor.protocolVersion(2);
  delay(100);
  
  // check if we can communicate with the motor
  // if not, we stop here
  uint8_t status = motor.init();
  if(status != DYN_STATUS_OK)
  {
    while(1);
  }

  motor.enableTorque();

  // set to joint mode, with a 180° angle range
  // see robotis doc to compute angle values
  motor.jointMode(204, 820);
  motor.speed(speed);
}

void loop() 
{
  // go to middle position
  motor.goalPosition(512);
  delay(500);

  // move 45° CCW
  motor.goalPosition(666);
  delay(500);

  // go to middle position
  motor.goalPosition(512);
  delay(500);

  // move 45° CW
  motor.goalPosition(358);
  delay(500);
}
