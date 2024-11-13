
#ifndef DYNAMIXEL_INTERFACE_IMPL_H
#define DYNAMIXEL_INTERFACE_IMPL_H

#include "DynamixelInterface2.h"
#include <Arduino.h>
//#include <SoftwareSerial.h>

template <class T>
class DynamixelInterfaceImpl : public DynamixelInterface
{
private:
    /** \brief Switch stream to read (receive)) mode */
    void readMode();

    /** \brief Switch stream to write (send) mode */
    void writeMode();

    unsigned long baud;

public:
    /**
     * \brief Constructor
     * \param[in] aStreamController : stream controller that abstract real stream
     * \param[in] aTxPin : pin number of the tx pin
     */
    DynamixelInterfaceImpl(T &aStream, uint8_t aTxPin);

    /**
     * \brief Destructor
     * Delete stream if it is owned by the instance
     */
    ~DynamixelInterfaceImpl();

    /**
     * \brief Start interface
     * \param[in] aBaud : Baudrate
     *
     * Start the interface with desired baudrate, call once before using the interface
     */
    void begin(unsigned long aBaud);

    /**
     * \brief Change timeOut
     * \param[in] timeOut : ms
     */
    void setTimeOut(uint16_t timeOut);

    /**
     * \brief Hardware specific preparations before transaction
     *
     * Reinit interface for dynamixel (flush, set baudrate)
     * for case when interface shared with other functional
     */
    void prepareTransaction();

    /**
     * \brief Hardware specific cleaning after transaction
     *
     * Reinit interface to its original state (flush, set baudrate)
     * for case when interface shared with other functional
     */
    void endTransaction();

    /**
     * \brief Send a packet on bus
     * \param[in] aPacket : Packet to send
     *
     * The function wait for the packet to be completly sent (using Stream.flush)
     */
    void sendPacket(const DynamixelPacket &aPacket);

        /**
     * \brief Send a packet on bus
     * \param[in] aPacket : Packet to send
     *
     * The function wait for the packet to be completly sent (using Stream.flush)
     */
    void sendPacket2(DynamixelPacket2 &aPacket);

    /**
     * \brief Receive a packet on bus
     * \param[out] aPacket : Received packet. mData field must be previously allocated
     * \param[in] answerSize : the size of the memory allocated to the mData field
     *
     * The function wait for a new packet on the bus.
     * Timeout depends of timeout of the underlying stream.
     * Return error code in case of communication error (timeout, checksum error, ...)
     */
    void receivePacket(DynamixelPacket &aPacket, uint8_t answerSize = 0);

        /**
     * \brief Receive a packet on bus
     * \param[out] aPacket : Received packet. mData field must be previously allocated
     * \param[in] answerSize : the size of the memory allocated to the mData field
     *
     * The function wait for a new packet on the bus.
     * Timeout depends of timeout of the underlying stream.
     * Return error code in case of communication error (timeout, checksum error, ...)
     */
    void receivePacket2(DynamixelPacket2 &aPacket, uint16_t answerSize = 0 , uint8_t mode = RECEIVE_NORMAL);

    /**
     * \brief Stop interface
     */
    void end();

    static const uint8_t NO_DIR_PORT = 255;

private:
    T &mStream;

protected:
    const uint8_t mTxPin;
};

class HardwareDynamixelInterface : public DynamixelInterfaceImpl<HardwareSerial>
{
public:
    HardwareDynamixelInterface(HardwareSerial &aSerial);
    ~HardwareDynamixelInterface();
};

#endif
