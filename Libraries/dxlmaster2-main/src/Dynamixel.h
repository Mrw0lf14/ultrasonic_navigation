/**
 * \file Dynamixel.h
 * \brief Define classes for dynamixel protocol
*/

#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#include <stdint.h>
#include <stdlib.h> 

#define DXL_LOBYTE(w) ((uint8_t)((w) & 0xff))
#define DXL_HIBYTE(w) ((uint8_t)(((w) >> 8) & 0xff))

#define DXL_LOBYTE(w) ((uint8_t)((w) & 0xff))
#define DXL_HIBYTE(w) ((uint8_t)(((w) >> 8) & 0xff))

/** \brief ID for broadcast */
#define BROADCAST_ID 0xFE

#define NO_ADDRESS  0xFFFF
#define NO_DATA     0xFFFF

#define HEADERS_LEN  4
#define ID_LEN       1
#define LEN_LEN      2
#define INST_LEN     1
#define ERR_LEN      1
#define CRC_LEN      2

#define HEAD_LEN            (HEADERS_LEN + ID_LEN + LEN_LEN + INST_LEN)
#define RX_HEAD_BUFF        (HEAD_LEN)
#define TX_MIN_LENGTH       (INST_LEN + CRC_LEN)
#define RX_MIN_LENGTH       (INST_LEN + ERR_LEN + CRC_LEN)

#define PING_TX_PARAMS_LEN  0
#define PING_TX_LENGTH      (TX_MIN_LENGTH + PING_TX_PARAMS_LEN)
#define PING_RX_PARAMS_LEN  3
#define PING_RX_LENGTH      (RX_MIN_LENGTH + PING_RX_PARAMS_LEN)

#define READ_TX_PARAMS_LEN  4
#define READ_TX_LENGTH      (TX_MIN_LENGTH + READ_TX_PARAMS_LEN)
#define READ_RX_PARAMS_LEN  0
#define READ_RX_LENGTH      (RX_MIN_LENGTH + READ_RX_PARAMS_LEN)

#define WRITE_TX_PARAMS_LEN 2
#define WRITE_TX_LENGTH     (TX_MIN_LENGTH + WRITE_TX_PARAMS_LEN)
#define WRITE_RX_PARAM_LEN  0
#define WRITE_RX_LENGTH     (RX_MIN_LENGTH + WRITE_RX_PARAM_LEN)

#define ACTIO_TX_PARAMS_LEN 0
#define ACTIO_TX_LENGTH     (TX_MIN_LENGTH + ACTIO_TX_PARAMS_LEN)
#define ACTIO_RX_PARAM_LEN  0
#define ACTIO_RX_LENGTH     (RX_MIN_LENGTH + ACTIO_RX_PARAM_LEN)

#define RESET_TX_PARAMS_LEN 1
#define RESET_TX_LENGTH     (TX_MIN_LENGTH + RESET_TX_PARAMS_LEN)
#define RESET_RX_PARAM_LEN  0
#define RESET_RX_LENGTH     (RX_MIN_LENGTH + RESET_RX_PARAM_LEN)

#define REBOOT_TX_PARAMS_LEN 0
#define REBOOT_TX_LENGTH     (TX_MIN_LENGTH + REBOOT_TX_PARAMS_LEN)
#define REBOOT_RX_PARAM_LEN  0
#define REBOOT_RX_LENGTH     (RX_MIN_LENGTH + REBOOT_RX_PARAM_LEN)

#define CLEAR_TX_PARAMS_LEN 0
#define CLEAR_TX_LENGTH     (TX_MIN_LENGTH + CLEAR_TX_PARAMS_LEN)
#define CLEAR_RX_PARAM_LEN  0
#define CLEAR_RX_LENGTH     (RX_MIN_LENGTH + CLEAR_RX_PARAM_LEN)

#define BACKUP_TX_PARAMS_LEN 0
#define BACKUP_TX_LENGTH     (TX_MIN_LENGTH + BACKUP_TX_PARAMS_LEN)
#define BACKUP_RX_PARAM_LEN  0
#define BACKUP_RX_LENGTH     (RX_MIN_LENGTH + BACKUP_RX_PARAM_LEN)

#define SYNC_READ_TX_PARAMS_LEN 4
#define SYNC_READ_TX_LENGTH     (TX_MIN_LENGTH + SYNC_READ_TX_PARAMS_LEN)
#define SYNC_READ_RX_PARAM_LEN  0
#define SYNC_READ_RX_LENGTH     (RX_MIN_LENGTH + SYNC_READ_RX_PARAM_LEN)

#define SYNC_WRITE_TX_PARAMS_LEN 4
#define SYNC_WRITE_TX_LENGTH     (TX_MIN_LENGTH + SYNC_WRITE_TX_PARAMS_LEN)
#define SYNC_WRITE_RX_PARAM_LEN  0
#define SYNC_WRITE_RX_LENGTH     (RX_MIN_LENGTH + SYNC_WRITE_RX_PARAM_LEN)

#define FAST_SYNC_READ_TX_PARAMS_LEN 4
#define FAST_SYNC_READ_TX_LENGTH     (TX_MIN_LENGTH + FAST_SYNC_READ_TX_PARAMS_LEN)
#define FAST_SYNC_READ_RX_PARAM_LEN  0
#define FAST_SYNC_READ_RX_LENGTH     (RX_MIN_LENGTH + FAST_SYNC_READ_RX_PARAM_LEN)

#define BULK_READ_TX_PARAMS_LEN 0
#define BULK_READ_TX_LENGTH     (TX_MIN_LENGTH + BULK_READ_TX_PARAMS_LEN)
#define BULK_READ_RX_PARAM_LEN  0
#define BULK_READ_RX_LENGTH     (RX_MIN_LENGTH + BULK_READ_RX_PARAM_LEN)

#define BULK_WRITE_TX_PARAMS_LEN 0
#define BULK_WRITE_TX_LENGTH     (TX_MIN_LENGTH + BULK_WRITE_TX_PARAMS_LEN)
#define BULK_WRITE_RX_PARAM_LEN  0
#define BULK_WRITE_RX_LENGTH     (RX_MIN_LENGTH + BULK_WRITE_RX_PARAM_LEN)



/** \brief Type of dynamixel device ID */
typedef uint8_t DynamixelID;
/** \brief Type of dynamixel status code */
typedef uint8_t DynamixelStatus;
/** \brief Type of dynamixel instructions */
typedef uint8_t DynamixelInstruction;


/** \brief Protocol version of dynamixel device ID */
enum DynProtocol
{
    DYN_PROTOCOL_V1 = 0x01,
	DYN_PROTOCOL_V2 = 0x02,
};

/** \brief Communication mode */
enum CommMode
{
    RECEIVE_NORMAL = 0x00,
	RECEIVE_FAST = 0x01,
};

/**
 * \brief Dynamixel intruction values
*/
enum DynInstruction
{
	DYN_PING		    = 0x01,
	DYN_READ		    = 0x02,
	DYN_WRITE		    = 0x03,
	DYN_REG_WRITE	    = 0x04,
	DYN_ACTION		    = 0x05,
	DYN_RESET		    = 0x06,
    DYN_REBOOT		    = 0x08,
    DYN_CLEAR		    = 0x10,
    DYN_CTBACKUP	    = 0x20,
    DYN_RSTATUS	        = 0x55,
    DYN_SYNC_READ	    = 0x82,
    DYN_SYNC_WRITE	    = 0x83,
    DYN_FAST_SYNK_READ	= 0x8A,
    DYN_BULK_READ	    = 0x92,
    DYN_BULK_WRITE	    = 0x93,
    DYN_FAST_BULK_READ	= 0x9A
};

/**
 * \brief Dynamixel intruction values for V2
*/
enum DynInstruction2
{
	INST_PING		    = 0x01,
	INST_READ		    = 0x02,
	INST_WRITE		    = 0x03,
	INST_REG_WRITE	    = 0x04,
	INST_ACTION		    = 0x05,
	INST_RESET		    = 0x06,
    INST_REBOOT		    = 0x08,
    INST_CLEAR		    = 0x10,
    INST_BACKUP	        = 0x20,
    INST_RSTATUS	    = 0x55,
    INST_SYNC_READ	    = 0x82,
    INST_SYNC_WRITE	    = 0x83,
    INST_FAST_SYNK_READ	= 0x8A,
    INST_BULK_READ	    = 0x92,
    INST_BULK_WRITE	    = 0x93,
    INST_FAST_BULK_READ	= 0x9A
};

/**
 * \brief Dynamixel status values
 *
 * How to interpret status value :
 *
 * If (status & DYN_STATUS_COM_ERROR) == 0 , the value is the 
 * the status returned by the motor, describing its internal
 * error.
 * If (status & DYN_STATUS_COM_ERROR) == 1, there was an error during
 * communication, and the value describe that error.
 *
 * DYN_STATUS_CHECKSUM_ERROR may appear in both cases, in the first
 * case, it means there was an error in the checksum of the 
 * instruction packet, in second case in the response packet.
*/
enum DynStatus
{
	DYN_STATUS_OK				    = 0,
	DYN_STATUS_INPUT_VOLTAGE_ERROR	= 1,
	DYN_STATUS_ANGLE_LIMIT_ERROR	= 2,
	DYN_STATUS_OVERHEATING_ERROR	= 4,
	DYN_STATUS_RANGE_ERROR			= 8,
	DYN_STATUS_CHECKSUM_ERROR		= 16,
	DYN_STATUS_OVERLOAD_ERROR		= 32,
	DYN_STATUS_INSTRUCTION_ERROR	= 64,

	DYN_STATUS_TIMEOUT			    = 1,
	DYN_STATUS_COM_ERROR			= 128,
	DYN_STATUS_INTERNAL_ERROR		= 255
};

enum DynV2_Status
{
	DYN2_STATUS_OK	            = 0x00,
	DYN2_STATUS_RESULT_FAIL	    = 0x01, // Failed to process the sent Instruction Packet
	DYN2_STATUS_INSTRUCTION_ERR	= 0x02, // Undefined Instruction has been used, Action has been used without Reg Write
	DYN2_STATUS_CRC_ERR	        = 0x03, // CRC of the sent Packet does not match
	DYN2_STATUS_DATA_RANGE_ERR  = 0x04, // Data to be written in the corresponding Address is outside the range of the minimum/maximum value
	DYN2_STATUS_DATA_LENGHT_ERR	= 0x05, // Attempt to write Data that is shorter than the data length of the corresponding Address
	DYN2_STATUS_DATA_LIMIT_ERR	= 0x06, // Data to be written in the corresponding Address is outside of the Limit value
	DYN2_STATUS_ACCESS_ERR	    = 0x07, // Attempt to read/write <> Write/Read Only or has not been defined or ROM lock

    DYN2_STATUS_TIMEOUT	        = 0x81,
    DYN2_STATUS_HEADERS_ERR	    = 0x82,
    DYN2_STATUS_PACKET_ID_ERR	= 0x83,
    DYN2_STATUS_PACKET_LEN_ERR	= 0x84,
    DYN2_STATUS_PACKET_INST_ERR	= 0x85,
    DYN2_STATUS_TIMEOUT_ERR     = 0x86,
    DYN2_STATUS_TIMEOUT_DATA	= 0x87,
    DYN2_STATUS_TIMEOUT_CRC	    = 0x88,
    DYN2_STATUS_PACKET_CRC_ERR	= 0x89,
};


/**
 * \brief Dynamixel control table addresses (only addresses used by all models)
*/
enum DynCommonAddress
{
	/** \brief Model number, uint16_t , read only */
	DYN_ADDRESS_MODEL		= 0x00,
	/** \brief Firmware version, uint8_t, read only */
	DYN_ADDRESS_FIRMWARE	= 0x02,
	/** \brief Device ID, uint8_t, writable */
	DYN_ADDRESS_ID			= 0x03,
	/** \brief Communication baudrate, uint8_t, writable */
	DYN_ADDRESS_BAUDRATE	= 0x04,
	/** \brief Return Delay Time , uint8_t, writable */
	DYN_ADDRESS_RDT			= 0x05,
	/** \brief Status Return Level , uint8_t, writable 
	 *
	 * Define when the device will send back a status packet :
	 * 0 : Ping only
	 * 1 : Read and ping
	 * 2 : All instructions
	*/
	DYN_ADDRESS_SRL			= 0x10 
};

/**
 * \brief Dynamixel motor control table addresses 
 *        (only addresses used by all motor models)
*/
enum DynMotorAddress
{
	/** \brief Clockwise angle limit, uint16_t , writable */
	DYN_ADDRESS_CW_LIMIT		= 0x06,
	/** \brief Counnter clockwise angle limit, uint16_t , writable */
	DYN_ADDRESS_CCW_LIMIT		= 0x08,
	/** \brief Maximum torque, uint16_t , writable */
	DYN_ADDRESS_MAX_TORQUE		= 0x0E,
	/** \brief Enable torque, uint8_t , writable */
	DYN_ADDRESS_ENABLE_TORQUE	= 0x18,
	/** \brief LED state, uint8_t , writable */
	DYN_ADDRESS_LED			    = 0x19,
	/** \brief CW compliance margin, uint8_t , writable */
	DYN_ADDRESS_CW_COMP_MARGIN	= 0x1A,
	/** \brief CCW compliance margin, uint8_t , writable */
	DYN_ADDRESS_CCW_COMP_MARGIN = 0x1B,
	/** \brief CW compliance slope, uint8_t , writable */
	DYN_ADDRESS_CW_COMP_SLOPE	= 0x1C,
	/** \brief CCW compliance slope, uint8_t , writable */
	DYN_ADDRESS_CCW_COMP_SLOPE  = 0x1D,
	/** \brief Goal position, uint16_t , writable */
	DYN_ADDRESS_GOAL_POSITION	= 0x1E,
	/** \brief Goal speed, uint16_t , writable */
	DYN_ADDRESS_GOAL_SPEED		= 0x20,

	DYN_ADDRESS_TORQUE_LIMIT	= 0x22,
	/** \brief Current position, uint16_t , readable */
	DYN_ADDRESS_CURRENT_POSITION = 0x24,
	/** \brief Nonzero if any movement, uint8_t, readable */
	DYN_ADDRESS_MOVING		    = 0x2E
};

enum Dyn_ver2_MotorAddress
{
    DYN2_ADDR_OPERATION_MODE	= 11,
    DYN2_ADDR_MAX_POS_LIMIT		= 48,
	DYN2_ADDR_MIN_POS_LIMIT		= 52,

	DYN2_ADDR_ENABLE_TORQUE	    = 64,
	DYN2_ADDR_LED			    = 65,

	DYN2_ADDR_GOAL_SPEED		= 104,
	DYN2_ADDR_PROFILE_VELOCITY  = 112,
	DYN2_ADDR_GOAL_POSITION	    = 116,
    DYN2_ADDR_MOVING		    = 122,
	DYN2_ADDR_CURRENT_POSITION  = 132,

};

/**
 * \brief Dynamixel model number values
*/
enum DynModel
{
	DYN_MODEL_AX12A	= 0x0C,
	DYN_MODEL_AX12W	= 0x2C,
	DYN_MODEL_AX18A	= 0x12,
	 
	DYN_MODEL_DX113	= 0x71,
	DYN_MODEL_DX114	= 0x74,
	DYN_MODEL_DX117	= 0x75,
	 
	DYN_MODEL_RX10	= 0x0A,
	DYN_MODEL_RX24F	= 0x18,
	DYN_MODEL_RX28	= 0x1C,
	DYN_MODEL_RX64	= 0x40,
	 
	DYN_MODEL_EX106	= 0x6B,
	 
	DYN_MODEL_MX12W	= 0x68,
	DYN_MODEL_MX28T	= 0x1D,
	DYN_MODEL_MX28R	= 0x1D,
	DYN_MODEL_MX64T	= 0x36,
	DYN_MODEL_MX64R	= 0x36,
	DYN_MODEL_MX106T= 0x40,
	DYN_MODEL_MX106R= 0x40,
	 
	DYN_MODEL_AXS1	= 0x0D
};


/**
 * \struct DynamixelPacket Protocol 1.0
 * \brief Struct of a dynamixel packet (instruction or status)
*/
struct DynamixelPacket
{
    DynamixelPacket() {}
    // note : allow to constuct from const data, but const_cast it (constness should be respected if code is correct)
    DynamixelPacket(
        uint8_t aID, 
        uint8_t aInstruction, 
        uint8_t aLength, 
        const uint8_t *aData, 
        uint8_t aAddress = 255, 
        uint8_t aDataLength = 255, 
        uint8_t aIDListSize = 0, 
        const uint8_t *aIDList = NULL) 
        : mID(aID), 
          mIDListSize(aIDListSize), 
          mIDList(const_cast<uint8_t *>(aIDList)), 
          mLength(aLength), 
          mInstruction(aInstruction), 
          mAddress(aAddress), 
          mDataLength(aDataLength), 
          mData(const_cast<uint8_t *>(aData))
    {
        mCheckSum = checkSum();
    }

    /** \brief Packet ID */
	DynamixelID mID;
	/** \brief ID list, used for sync write, set to 0 if not used */
	uint8_t mIDListSize;
	DynamixelID *mIDList;
	/** \brief Packet length (full length)*/
	uint8_t mLength;
	/** \brief Packet instruction or status */
	union{
		DynamixelInstruction mInstruction;
		DynamixelStatus mStatus;
	};
	/** \brief Address to read/write, set to 255 if not used */
	uint8_t mAddress;
	/** \brief Length of data to read/write, only needed for read and sync write, set to 255 if not used */
	uint8_t mDataLength;
	/** \brief Pointer to packet parameter (or NULL if no parameter) */
	uint8_t *mData;
	/** \brief Packet checksum */
	uint8_t mCheckSum;
	
	/**
	 * \brief Compute checksum of the packet
	 * \return Checksum value
	*/
	uint8_t checkSum();
};

/**
 * \struct DynamixelPacket Protocol 2.0
 * \brief Struct of a dynamixel packet (instruction or status)
*/
struct DynamixelPacket2
{
	DynamixelPacket2(){}
    
	/* note : allow to constuct from const data, but const_cast it 
       (constness should be respected if code is correct) */
    DynamixelPacket2(
        uint8_t aID,
        uint16_t aLength,
        uint8_t aInstruction,
        const uint8_t *aParams = NULL,
        uint16_t aParamSize = 0,
        const uint8_t *aRxData = NULL,
        uint16_t aRxDataSize = 0)
        : mID(aID),
          mTxLength(aLength),
          mInstruction(aInstruction),
          mParams(const_cast<uint8_t *>(aParams)),
          mParamSize(aParamSize),
          mRxData(const_cast<uint8_t *>(aRxData)),
          mRxDataLength(aRxDataSize)
    {
        mHead[0] = 0xFF;
        mHead[1] = 0xFF;
        mHead[2] = 0xFD;
        mHead[3] = 0x00;
        mHead[4] = mID;
        mHead[5] = (DXL_LOBYTE(mTxLength));
        mHead[6] = (DXL_HIBYTE(mTxLength));
        mHead[7] = mInstruction;

        mCheckSum = updateCRC(0, mHead, HEAD_LEN);
        mCheckSum = updateCRC(mCheckSum, mParams, mParamSize);
    }

    uint8_t mHead[HEAD_LEN];
	DynamixelID mID;
	uint16_t mTxLength;
	union{
		DynamixelInstruction mInstruction;
		DynamixelStatus mStatus;
	};
	uint8_t *mParams;
	uint16_t mParamSize;

	uint8_t *mRxData;
	uint16_t mRxDataLength;

	uint16_t mCheckSum;
	
    uint16_t updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, 
                                           uint16_t data_blk_size);

};

#endif