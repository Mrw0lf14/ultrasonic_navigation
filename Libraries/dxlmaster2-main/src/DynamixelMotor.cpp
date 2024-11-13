#include "DynamixelMotor.h"

DynamixelMotor::DynamixelMotor(DynamixelID aId, uint8_t aVersion) : DynamixelDevice(aId, aVersion)
{
}

void DynamixelMotor::wheelMode()
{
    jointMode(0, 0);
}

void DynamixelMotor::jointMode(uint16_t aCWLimit, uint16_t aCCWLimit)
{
    if (mVer == DYN_PROTOCOL_V1) 
    {
        uint32_t data = (aCWLimit | (uint32_t(aCCWLimit) << 16));
        write(DYN_ADDRESS_CW_LIMIT, data); 
    }
    else
    {
        write(DYN2_ADDR_MIN_POS_LIMIT, aCWLimit);
        write(DYN2_ADDR_MAX_POS_LIMIT, aCCWLimit);  
    }
}

void DynamixelMotor::enableTorque(bool aTorque)
{
    if (mVer == DYN_PROTOCOL_V1) 
        write(DYN_ADDRESS_ENABLE_TORQUE, uint8_t(aTorque ? 1 : 0));
    else
        write(DYN2_ADDR_ENABLE_TORQUE, uint8_t(aTorque ? 1 : 0));
}

void DynamixelMotor::speed(int16_t aSpeed)
{
    //if (aSpeed < 0)
    //    aSpeed = -aSpeed | 1024;

    if (mVer == DYN_PROTOCOL_V1) 
        write(DYN_ADDRESS_GOAL_SPEED, aSpeed);
    else
        write(DYN2_ADDR_GOAL_SPEED, (uint32_t)aSpeed);
}

void DynamixelMotor::goalPosition(uint16_t aPosition)
{
    if (mVer == DYN_PROTOCOL_V1) 
        write(DYN_ADDRESS_GOAL_POSITION, aPosition);
    else
        write(DYN2_ADDR_GOAL_POSITION, aPosition);
    
}

void DynamixelMotor::led(uint8_t aState)
{ 
    if (mVer == DYN_PROTOCOL_V1) 
        write(DYN_ADDRESS_LED, aState);
    else
        write(DYN2_ADDR_LED, aState);
}

uint16_t DynamixelMotor::currentPosition()
{
    uint16_t currentPosition;

    if (mVer == DYN_PROTOCOL_V1) 
        read(DYN_ADDRESS_CURRENT_POSITION, currentPosition);
    else
        read(DYN2_ADDR_CURRENT_POSITION, currentPosition);

    return currentPosition; 
}

DynamixelStatus DynamixelMotor::getCurrentPosition(uint16_t &pos)
{
    if (mVer == DYN_PROTOCOL_V1) 
        return read(DYN_ADDRESS_CURRENT_POSITION, pos);
    else
        return read(DYN2_ADDR_CURRENT_POSITION, pos);
}

DynamixelStatus DynamixelMotor::setComplianceMargins(uint8_t cw_margin, uint8_t ccw_margin, uint8_t cw_slope, uint8_t ccw_slope)
{
    DynamixelStatus status;

    if (mVer == DYN_PROTOCOL_V2) 
        return 1;

    status = write(DYN_ADDRESS_CW_COMP_MARGIN, cw_margin);
    if (DYN_STATUS_OK != status)
    {
        return status;
    }

    status = write(DYN_ADDRESS_CCW_COMP_MARGIN, ccw_margin);
    if (DYN_STATUS_OK != status)
    {
        return status;
    }

    status = write(DYN_ADDRESS_CW_COMP_SLOPE, cw_slope);
    if (DYN_STATUS_OK != status)
    {
        return status;
    }

    return write(DYN_ADDRESS_CCW_COMP_SLOPE, ccw_slope);
}

DynamixelStatus DynamixelMotor::getComplianceMargins(uint8_t &cw_margin, uint8_t &ccw_margin, uint8_t &cw_slope, uint8_t &ccw_slope)
{
    DynamixelStatus status;

    if (mVer == DYN_PROTOCOL_V2) 
        return 1;

    status = read(DYN_ADDRESS_CW_COMP_MARGIN, cw_margin);
    if (DYN_STATUS_OK != status)
    {
        return status;
    }

    status = read(DYN_ADDRESS_CCW_COMP_MARGIN, ccw_margin);
    if (DYN_STATUS_OK != status)
    {
        return status;
    }

    status = read(DYN_ADDRESS_CW_COMP_SLOPE, cw_slope);
    if (DYN_STATUS_OK != status)
    {
        return status;
    }

    return read(DYN_ADDRESS_CCW_COMP_SLOPE, ccw_slope);
}

DynamixelStatus DynamixelMotor::isMoving(uint8_t &moving)
{
    if (mVer == DYN_PROTOCOL_V1) 
        return read(DYN_ADDRESS_MOVING, moving);
    else
        return read(DYN2_ADDR_MOVING, moving);
}
