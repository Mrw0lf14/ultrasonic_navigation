#ifndef DYNAMIXEL_CONSOLE_H
#define DYNAMIXEL_CONSOLE_H

#include "DynamixelInterface2.h"

extern HardwareDynamixelInterface DxlMaster;

class DynamixelConsole;

struct DynamixelCommand
{
    typedef DynamixelStatus (DynamixelConsole::*FunPtr)(int, char **);
    const char *mName;
    // const char *mDescription;
    FunPtr mCallback;
};

class Stream;
class DynamixelConsole
{
public:
    DynamixelConsole(Stream &aConsole, uint8_t aVersion = DYN_PROTOCOL_V1);
    void loop();

private:
    void run();
    int parseCmd(char **argv);
    void printStatus(DynamixelStatus aStatus);
    void printData(const uint8_t *data, uint8_t length);
    void printHelp();

    DynamixelStatus ping(int argc, char **argv);
    DynamixelStatus read(int argc, char **argv);
    DynamixelStatus write(int argc, char **argv);
    DynamixelStatus reset(int argc, char **argv);
    DynamixelStatus reg_write(int argc, char **argv);
    DynamixelStatus action(int argc, char **argv);
    DynamixelStatus sync_write(int argc, char **argv);
    DynamixelStatus baudrate(int argc, char **argv);

    const static size_t sLineBufferSize = 255;
    char mLineBuffer[sLineBufferSize + 1];
    char *mLinePtr;
    Stream &mConsole;
    uint8_t mVer; 

    const static DynamixelCommand sCommand[];
};

#endif
