#include "DynamixelInterfaceArduinoImpl.h"

// define TXEN, RXEN and RXCIE
#if __AVR__
#	if !defined(TXEN)
#		if defined(TXEN0)
#			define TXEN TXEN0
#			define RXEN RXEN0
#			define RXCIE RXCIE0
#		elif defined(TXEN1) // Some devices have uart1 but no uart0 (leonardo)
#			define TXEN TXEN1
#			define RXEN RXEN1
#			define RXCIE RXCIE1
#		endif
#	endif
#endif //__AVR__

// Set UART direction pins so that serial port messages and other devices on line do not interfere
#if defined(__AVR_ATmega2560__)
#	ifndef DXL_DIR_TXD_PIN
#		define DXL_DIR_TXD_PIN 42
#	endif
#	ifndef DXL_DIR_RXD_PIN
#		define DXL_DIR_RXD_PIN 43
#	endif
#elif defined(__AVR_ATmega328P__)
#	ifndef DXL_DIR_TXD_PIN
#		define DXL_DIR_TXD_PIN 2
#	endif
#	ifndef DXL_DIR_RXD_PIN
#		define DXL_DIR_RXD_PIN 2
#	endif
#elif defined(ESP32)
#	ifndef DXL_DIR_TXD_PIN
#		define DXL_DIR_TXD_PIN 5
#	endif
#	ifndef DXL_DIR_RXD_PIN
#		define DXL_DIR_RXD_PIN 5
#	endif
#endif

#if defined (__AVR_ATmega2560__)
	HardwareDynamixelInterface DxlMaster(Serial1);
#elif defined(ESP32)
    HardwareDynamixelInterface DxlMaster(Serial);
#elif defined(__AVR_ATmega328P__)
	HardwareDynamixelInterface DxlMaster(Serial);
#endif

template<class T>
void DynamixelInterfaceImpl<T>::readMode()
{
#if DXL_DIR_TXD_PIN != DXL_DIR_RXD_PIN
	digitalWrite(DXL_DIR_TXD_PIN, LOW);
#endif
	digitalWrite(DXL_DIR_RXD_PIN, HIGH);
}
	
template<class T>
void DynamixelInterfaceImpl<T>::writeMode()
{
	digitalWrite(DXL_DIR_RXD_PIN, LOW);
#if DXL_DIR_TXD_PIN != DXL_DIR_RXD_PIN
	digitalWrite(DXL_DIR_TXD_PIN, HIGH);
#endif
}
	
template<class T>
DynamixelInterfaceImpl<T>::DynamixelInterfaceImpl(T &aStream, uint8_t aTxPin):
	mStream(aStream), mTxPin(aTxPin)
{
	readMode();
	pinMode(DXL_DIR_TXD_PIN, OUTPUT);
	pinMode(DXL_DIR_RXD_PIN, OUTPUT);
}

template <class T>
DynamixelInterfaceImpl<T>::~DynamixelInterfaceImpl()
{
}

template <class T>
void DynamixelInterfaceImpl<T>::begin(unsigned long aBaud)
{
#if defined(ESP32)
    mStream.begin(115200);
#else
    mStream.begin(aBaud);
#endif

    // mStream.write(0);
    // Serial.println("Hello");
    baud = aBaud;
    mStream.setTimeout(3); // warning : response delay seems much higher than expected for some operation (eg writing eeprom)
    readMode();
}

template <class T>
void DynamixelInterfaceImpl<T>::setTimeOut(uint16_t timeOut)
{
    mStream.setTimeout(timeOut);
}

template<class T>
void DynamixelInterfaceImpl<T>::prepareTransaction()
{
#if defined(ESP32)
	mStream.flush();
	mStream.updateBaudRate(baud);
#endif
}

template<class T>
void DynamixelInterfaceImpl<T>::endTransaction()
{
#if defined(ESP32)
	mStream.flush();
	mStream.updateBaudRate(115200);
#endif
}

template <class T>
void DynamixelInterfaceImpl<T>::sendPacket(const DynamixelPacket &aPacket)
{
    mStream.flush();
    writeMode();
    // empty receive buffer, in case of a error in previous transaction
    while (mStream.available())
        mStream.read();
    mStream.write(0xFF);
    mStream.write(0xFF);
    mStream.write(aPacket.mID);
    mStream.write(aPacket.mLength);
    mStream.write(aPacket.mInstruction);
    uint8_t n = 0;
    if (aPacket.mAddress != 255)
    {
        mStream.write(aPacket.mAddress);
        ++n;
    }
    if (aPacket.mDataLength != 255)
    {
        mStream.write(aPacket.mDataLength);
        ++n;
    }
    if (aPacket.mLength > (n + 2U))
    {
        if (aPacket.mIDListSize == 0)
        {
            mStream.write(aPacket.mData, aPacket.mLength - 2 - n);
        }
        else
        {
            uint8_t *ptr = aPacket.mData;
            for (uint8_t i = 0; i < aPacket.mIDListSize; ++i)
            {
                mStream.write(aPacket.mIDList[i]);
                mStream.write(ptr, aPacket.mDataLength);
                ptr += aPacket.mDataLength;
            }
        }
    }
    mStream.write(aPacket.mCheckSum);
    mStream.flush();
    readMode();
}

template<class T>
void DynamixelInterfaceImpl<T>::sendPacket2(DynamixelPacket2 &aPacket)
{
	mStream.flush();
	writeMode();

	// empty receive buffer, in case of a error in previous transaction
	while(mStream.available()) {
		mStream.read();
    }

    mStream.write(aPacket.mHead, HEAD_LEN);
    mStream.write(aPacket.mParams, aPacket.mParamSize);
    mStream.write(DXL_LOBYTE(aPacket.mCheckSum));
    mStream.write(DXL_HIBYTE(aPacket.mCheckSum));

	mStream.flush();
   
	readMode();
}

template <class T>
void DynamixelInterfaceImpl<T>::receivePacket(DynamixelPacket &aPacket, uint8_t answerSize)
{
    static uint8_t buffer[3];

    aPacket.mIDListSize = 0;
    aPacket.mAddress = 255;
    aPacket.mDataLength = 255;
    if (mStream.readBytes(buffer, 2) < 2)
    {
        aPacket.mStatus = DYN_STATUS_COM_ERROR | DYN_STATUS_TIMEOUT;
        return;
    }
    if (buffer[0] != 255 || buffer[1] != 255)
    {
        aPacket.mStatus = DYN_STATUS_COM_ERROR;
        return;
    }
    if (mStream.readBytes(buffer, 3) < 3)
    {
        aPacket.mStatus = DYN_STATUS_COM_ERROR | DYN_STATUS_TIMEOUT;
        return;
    }
    if (aPacket.mID != buffer[0])
    {
        aPacket.mStatus = DYN_STATUS_COM_ERROR;
        return;
    }
    aPacket.mLength = buffer[1];
    if ((aPacket.mLength - 2) != answerSize)
    {
        aPacket.mStatus = DYN_STATUS_COM_ERROR;
        return;
    }
    aPacket.mStatus = buffer[2];
    if (aPacket.mLength > 2 
        && (int)mStream.readBytes(reinterpret_cast<char *>(aPacket.mData), aPacket.mLength - 2) < (int)(aPacket.mLength - 2))
    {
        aPacket.mStatus = DYN_STATUS_COM_ERROR | DYN_STATUS_TIMEOUT;
        return;
    }
    if (mStream.readBytes(reinterpret_cast<char *>(&(aPacket.mCheckSum)), 1) < 1)
    {
        aPacket.mStatus = DYN_STATUS_COM_ERROR | DYN_STATUS_TIMEOUT;
        return;
    }
    if (aPacket.checkSum() != aPacket.mCheckSum)
    {
        aPacket.mStatus = DYN_STATUS_COM_ERROR | DYN_STATUS_CHECKSUM_ERROR;
    }
}

template <class T>
void DynamixelInterfaceImpl<T>::receivePacket2(DynamixelPacket2 &aPacket, uint16_t answerSize, uint8_t mode)
{   
    uint8_t buffer[RX_HEAD_BUFF] = {0};
    aPacket.mCheckSum = 0;

    if (mode != RECEIVE_FAST)
    {
        if (mStream.readBytes(buffer, RX_HEAD_BUFF) < RX_HEAD_BUFF)
        {
            aPacket.mStatus = DYN2_STATUS_TIMEOUT;
            return;
        }

        if (buffer[0] != 0xFF || buffer[1] != 0xFF || buffer[2] != 0xFD || buffer[3] != 0x00)
        {
            aPacket.mStatus = DYN2_STATUS_HEADERS_ERR;
            return;
        }

        if (aPacket.mID != buffer[4])
        {
            aPacket.mStatus = DYN2_STATUS_PACKET_ID_ERR;
            return;
        }

        uint16_t rxLength = buffer[5] | ((uint16_t)buffer[6] << 8);
        if (rxLength != answerSize)
        {
            aPacket.mStatus = DYN2_STATUS_PACKET_LEN_ERR;
            return;
        }

        if (0x55 != buffer[7])
        {
            aPacket.mStatus = DYN2_STATUS_PACKET_INST_ERR;
            return;
        }

        aPacket.mCheckSum = aPacket.updateCRC(0, buffer, RX_HEAD_BUFF);
    }


    // Read Err field
    if (mStream.readBytes(buffer, ERR_LEN) < ERR_LEN) 
    {
        aPacket.mStatus = DYN2_STATUS_TIMEOUT_ERR;
        return;
    }

    aPacket.mStatus = buffer[0];
    aPacket.mCheckSum = aPacket.updateCRC(aPacket.mCheckSum, buffer, ERR_LEN);


    /* Read Data fields*/
    if (mStream.readBytes(aPacket.mRxData, aPacket.mRxDataLength) < aPacket.mRxDataLength)
    {
        aPacket.mStatus = DYN2_STATUS_TIMEOUT_DATA;
        return;
    }

    aPacket.mCheckSum = aPacket.updateCRC(aPacket.mCheckSum, aPacket.mRxData, aPacket.mRxDataLength);

    
    if (mStream.readBytes(buffer, CRC_LEN) < CRC_LEN)
    {
        aPacket.mStatus = DYN2_STATUS_TIMEOUT_CRC;
        return;
    }

    uint16_t rx_crc = ((uint16_t)buffer[1] << 8) | buffer[0];
    if (aPacket.mCheckSum != rx_crc)
    {
        aPacket.mStatus = DYN2_STATUS_PACKET_CRC_ERR;
        return;
    }

}

template<class T>
void DynamixelInterfaceImpl<T>::end()
{
	mStream.end();
}

template class DynamixelInterfaceImpl<HardwareSerial>;
//template class DynamixelInterfaceImpl<SoftwareSerial>;

//determine txpin number from hardware serial interface
uint8_t TxPinFromHardwareSerial(const HardwareSerial &aSerial)
{
#if defined ARDUINO_AVR_UNO || defined ARDUINO_AVR_DUEMILANOVE || defined ARDUINO_AVR_NANO || defined ARDUINO_AVR_MEGA2560 || defined ARDUINO_AVR_MEGA || defined ARDUINO_AVR_ADK || defined ARDUINO_AVR_MINI || defined ARDUINO_AVR_ETHERNET || defined ARDUINO_AVR_BT || defined ARDUINO_AVR_PRO
	if((&aSerial)==(&Serial))
		return 1;
#endif
#if defined ARDUINO_AVR_LEONARDO || defined ARDUINO_AVR_MICRO || defined ARDUINO_AVR_ROBOT_CONTROL || defined ARDUINO_ARC32_TOOLS
	if((&aSerial)==(&Serial1))
		return 1;
#endif
#if defined ARDUINO_AVR_MEGA2560 || defined ARDUINO_AVR_MEGA || defined ARDUINO_AVR_ADK 
	if((&aSerial)==(&Serial1))
		return 18;
	if((&aSerial)==(&Serial2))
		return 16;
	if((&aSerial)==(&Serial3))
		return 14;
#endif
	return -1;
}


HardwareDynamixelInterface::HardwareDynamixelInterface(HardwareSerial &aSerial):
	DynamixelInterfaceImpl(aSerial, TxPinFromHardwareSerial(aSerial))
{}

HardwareDynamixelInterface::~HardwareDynamixelInterface()
{}

template<class T>
void setReadMode(T &aStream, uint8_t aTxPin);
template<class T>
void setWriteMode(T &aStream, uint8_t aTxPin);

#if __AVR__
namespace {
//dirty trick to access protected member
class HardwareSerialAccess:public HardwareSerial
{
	public:
	volatile uint8_t * const ucsrb(){return _ucsrb;}
};
}

template<>
void setReadMode<HardwareSerial>(HardwareSerial &aStream, uint8_t aTxPin)
{
	HardwareSerialAccess &stream=reinterpret_cast<HardwareSerialAccess&>(aStream);
	*(stream.ucsrb()) &= ~_BV(TXEN);
	*(stream.ucsrb()) |= _BV(RXEN);
	*(stream.ucsrb()) |= _BV(RXCIE);
	pinMode(aTxPin, INPUT_PULLUP);
}

template<>
void setWriteMode<HardwareSerial>(HardwareSerial &aStream, uint8_t mTxPin)
{
	HardwareSerialAccess &stream=reinterpret_cast<HardwareSerialAccess&>(aStream);
	*(stream.ucsrb()) &= ~_BV(RXEN);
	*(stream.ucsrb()) &= ~_BV(RXCIE);
	*(stream.ucsrb()) |= _BV(TXEN);
}

#elif __arc__

//Arduino 101 specific code

template<>
void setReadMode<HardwareSerial>(HardwareSerial &aStream, uint8_t mTxPin)
{
	//enable pull up to avoid noise on the line
	pinMode(1, INPUT);
	digitalWrite(1, HIGH);
	// disconnect UART TX and connect UART RX
	SET_PIN_MODE(16, GPIO_MUX_MODE);
	SET_PIN_MODE(17, UART_MUX_MODE);
	pinMode(aTxPin, INPUT_PULLUP);
}

template<>
void setWriteMode<HardwareSerial>(HardwareSerial &aStream, uint8_t mTxPin)
{
	// disconnect UART RX and connect UART TX
	SET_PIN_MODE(17, GPIO_MUX_MODE);
	SET_PIN_MODE(16, UART_MUX_MODE);
}

#endif


