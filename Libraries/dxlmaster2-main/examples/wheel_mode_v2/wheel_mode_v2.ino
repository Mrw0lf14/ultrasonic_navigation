// Test motor wheel mode

#include "DxlMaster2.h"

// id of the motor
const uint8_t id = 1;
// speed, between -1023 and 1023
int16_t speed = 200;
// communication baudrate
const long unsigned int baudrate = 57600;

uint8_t led_state = true;

DynamixelMotor motor(id, 2);

void setup() 
{
    DxlMaster.begin(baudrate);
    Serial.begin(57600);
    Serial.println("Hello!");
    delay(100);

    uint8_t buf[3];
    uint8_t status;
    status = motor.ping(buf);
    uint16_t num = buf[0] | (buf[1] << 8); 
    if (status == DYN_STATUS_OK) {
        Serial.print("\tOk!");
        Serial.print(" Number: ");
        Serial.print(num, HEX);
        Serial.print(" Version: ");
        Serial.print(buf[2], HEX);
    }
    else {
        Serial.print("\tErr!");
    }
    Serial.println(" Status = " + String(status));


    motor.enableTorque(0); 
    motor.write(DYN2_ADDR_OPERATION_MODE, (uint8_t)1); 
    motor.enableTorque(1); 
}

void loop() 
{
    motor.speed(speed);
    speed = -speed;
    motor.write(DYN2_ADDR_LED, (uint8_t)led_state);
    led_state = !led_state;
    delay(3000);
}