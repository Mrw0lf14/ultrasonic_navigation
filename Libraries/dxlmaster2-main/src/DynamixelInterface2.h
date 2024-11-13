#ifndef DYNAMIXEL_INTERFACE_V2_H
#define DYNAMIXEL_INTERFACE_V2_H

#include "Dynamixel.h"

/**
 * \class  DynamixelInterface
 * \brief Represent a dynamixel bus
 */
class DynamixelInterface
{
public:
    virtual void begin(unsigned long aBaud) = 0;
    virtual void sendPacket(const DynamixelPacket &aPacket) = 0;
    virtual void sendPacket2(DynamixelPacket2 &aPacket) = 0;
    virtual void receivePacket(DynamixelPacket &aPacket, uint8_t answerSize = 0) = 0;
    virtual void receivePacket2(DynamixelPacket2 &aPacket, uint16_t answerSize = 0, uint8_t mode = 0) = 0;
    virtual void prepareTransaction() = 0;
    virtual void endTransaction() = 0;
    virtual void end() = 0;

    void transaction(bool aExpectStatus, uint8_t answerSize = 0);
    void transaction2(bool aExpectStatus, uint16_t answerSize = 0);
    void transaction2_loop(uint16_t count, uint16_t answerSize);

    template <class T>
    inline DynamixelStatus read(uint8_t aVer, uint8_t aID,  uint16_t aAddress, T &aData, uint8_t aStatusReturnLevel = 2);
    template <class T>
    inline DynamixelStatus write(uint8_t aVer, uint8_t aID, uint16_t aAddress, const T &aData, uint8_t aStatusReturnLevel = 2);
    template <class T>
    inline DynamixelStatus regWrite(uint8_t aVer, uint8_t aID, uint16_t aAddress, const T &aData, uint8_t aStatusReturnLevel = 2);

    inline DynamixelStatus read(uint8_t aID, uint16_t aAddress, uint8_t aSize, uint8_t *aData, uint8_t aStatusReturnLevel = 2);
    inline DynamixelStatus write(uint8_t aID, uint16_t aAddress, uint8_t aSize, uint8_t *aData, uint8_t aStatusReturnLevel = 2);

    DynamixelStatus read(uint8_t aVer, uint8_t aID, uint16_t aAddress, uint16_t aRxSize, uint8_t *aRxBuf, uint8_t aStatusReturnLevel = 2);
    DynamixelStatus write(uint8_t aVer, uint8_t aID, uint16_t aAddress, uint16_t aTxSize, const uint8_t *aTxBuf, uint8_t aStatusReturnLevel = 2);
    DynamixelStatus regWrite(uint8_t aVer, uint8_t aID, uint16_t aAddress, uint16_t aTxSize, const uint8_t *aTxBuf, uint8_t aStatusReturnLevel = 2);
    
    DynamixelStatus syncRead(uint8_t aVer, uint8_t nID, const uint8_t *aID, uint16_t aAddress, uint16_t aSize, uint8_t *aRxBuf);
    DynamixelStatus syncWrite(uint8_t aVer, uint8_t nID, const uint8_t *aID, uint16_t aAddress, uint16_t aSize, const uint8_t *aTxBuf);
    DynamixelStatus fastSyncRead(uint8_t aVer, uint8_t nID, const uint8_t *aID, uint16_t aAddress, uint16_t aSize, uint8_t *aRxBuf);
    DynamixelStatus bulkRead(uint8_t aVer, uint8_t nID, const uint8_t *aTxBuf, uint16_t aSize, uint8_t *aRxBuf);
    DynamixelStatus bulkWrite(uint8_t aVer, uint8_t nID, uint16_t aTxSize, const uint8_t *aTxBuf);
    
    DynamixelStatus ping(uint8_t aVer, uint8_t aID, uint8_t *rxBuf = NULL);
    DynamixelStatus action(uint8_t aVer, uint8_t aID, uint8_t aStatusReturnLevel = 2);
    DynamixelStatus reset(uint8_t aVer, uint8_t aID, uint8_t aLvl = 0xff, uint8_t aStatusReturnLevel = 2);
    DynamixelStatus reboot(uint8_t aVer, uint8_t aID, uint8_t aStatusReturnLevel);
    DynamixelStatus clear(uint8_t aVer, uint8_t aID, uint16_t aTxSize, const uint8_t *aTxBuf, uint8_t aStatusReturnLevel = 2);
    DynamixelStatus backup(uint8_t aVer, uint8_t aID, uint16_t aTxSize, const uint8_t *aTxBuf, uint8_t aStatusReturnLevel = 2);

private:
    DynamixelPacket mPacket;
    DynamixelPacket2 mPacket2;
};


template <class T>
DynamixelStatus DynamixelInterface::read(uint8_t aVer, uint8_t aID, uint16_t aAddress, T &aData, uint8_t aStatusReturnLevel)
{
    return read(aVer, aID, aAddress, uint16_t(sizeof(T)), (uint8_t *)&aData, aStatusReturnLevel);
}

template <class T>
DynamixelStatus DynamixelInterface::write(uint8_t aVer, uint8_t aID, uint16_t aAddress, const T &aData, uint8_t aStatusReturnLevel)
{
    return write(aVer, aID, aAddress, uint16_t(sizeof(T)), (const uint8_t *)&aData, aStatusReturnLevel);
}

template <class T>
DynamixelStatus DynamixelInterface::regWrite(uint8_t aVer, uint8_t aID, uint16_t aAddress, const T &aData, uint8_t aStatusReturnLevel)
{
    return regWrite(aVer, aID, aAddress, uint16_t(sizeof(T)), (const uint8_t *)&aData, aStatusReturnLevel);
}


inline DynamixelStatus DynamixelInterface::read(uint8_t aID, uint16_t aAddress, uint8_t aSize, uint8_t *aData, uint8_t aStatusReturnLevel)
{
    return read(0x01, aID, aAddress, aSize, aData, aStatusReturnLevel);
}

inline DynamixelStatus DynamixelInterface::write(uint8_t aID, uint16_t aAddress, uint8_t aSize, uint8_t *aData, uint8_t aStatusReturnLevel)
{
    return write(0x01, aID, aAddress, aSize, aData, aStatusReturnLevel);
}




#if defined(ARDUINO)
#include "DynamixelInterfaceArduinoImpl.h"
#else
#error "Your platform is not supported"
#endif

#endif
