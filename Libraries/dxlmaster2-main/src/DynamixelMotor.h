/**
 * \file DynamixelMotor.h
 * \brief Define classe for dynamixel motor
 */

#ifndef DYNAMIXEL_MOTOR_H
#define DYNAMIXEL_MOTOR_H

#include "DynamixelDevice.h"

class DynamixelMotor : public DynamixelDevice
{
public:
    DynamixelMotor(DynamixelID aId, uint8_t aVersion = 1);

    void wheelMode();
    void jointMode(uint16_t aCWLimit = 0, uint16_t aCCWLimit = 0x3FF);

    void enableTorque(bool aTorque = true);
    void speed(int16_t aSpeed);
    void goalPosition(uint16_t aPosition);

    void led(uint8_t aState);

    uint16_t currentPosition();

    DynamixelStatus getCurrentPosition(uint16_t &pos);
    DynamixelStatus setComplianceMargins(uint8_t cw_margin, uint8_t ccw_margin, uint8_t cw_slope, uint8_t ccw_slope);
    DynamixelStatus getComplianceMargins(uint8_t &cw_margin, uint8_t &ccw_margin, uint8_t &cw_slope, uint8_t &ccw_slope);

    DynamixelStatus isMoving(uint8_t &moving);
};

#endif
