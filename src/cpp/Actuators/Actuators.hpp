#include "include/DynamixelSDK.h"

class Actuators
{
public:
    explicit Actuators(int dxlID);
    ~Actuators();

    bool setMovingSpeed(int speed);
    bool setGoalPosition(int goalPosition);

private:
    const char* DEVICE_NAME = "/dev/ttyACM0";
    const int BAUD_RATE = 115200;
    const int PROTOCOL_VERSION = 1;
    int dxlID;

    dynamixel::PortHandler* portHandler;
    dynamixel::PacketHandler* packetHandler;
};
