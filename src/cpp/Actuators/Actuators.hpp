#include "include/DynamixelSDK.h"

// The Actuators class provides methods to control Dynamixel actuators, it uses Dynamixel SDK
class Actuators
{
public:
    explicit Actuators(int dxlID);
    ~Actuators();

    bool setMovingSpeed(int speed);
    bool setGoalPosition(int goalPosition);

private:
    const char* DEVICE_NAME = "/dev/ttyACM0";   // Device name for the actuator
    const int BAUD_RATE = 115200;               // Baud rate for the actuator
    const int PROTOCOL_VERSION = 1;             // Protocol version for the actuator
    int dxlID;                                  // Dynamixel ID for the actuator

    dynamixel::PortHandler* portHandler;
    dynamixel::PacketHandler* packetHandler;
};
