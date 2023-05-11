#include "Actuators.hpp"
#include <iostream>

Actuators::Actuators(int dxlID)
    : dxlID(dxlID)
{
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!portHandler->openPort())
    {
        std::cerr << "Failed to open the port" << std::endl;
    }
    if (!portHandler->setBaudRate(BAUD_RATE))
    {
        std::cerr << "Failed to set the baud rate" << std::endl;
    }
}

Actuators::~Actuators()
{
    portHandler->closePort();
}

bool Actuators::setMovingSpeed(int speed)
{
    uint8_t dxl_error = 0;
    int result = packetHandler->write2ByteTxRx(portHandler, dxlID, 32, speed, &dxl_error);
    if (result != COMM_SUCCESS)
    {
        std::cerr << "Failed to write moving speed: " << packetHandler->getTxRxResult(result) << std::endl;
        return false;
    }
    else if (dxl_error != 0)
    {
        std::cerr << "Error writing moving speed: " << packetHandler->getRxPacketError(dxl_error) << std::endl;
        return false;
    }
    return true;
}

bool Actuators::setGoalPosition(int goalPosition)
{
    uint8_t dxl_error = 0;
    std::cout << "data from elevator id" <<dxlID<<"  angle " <<goalPosition << std::endl;
    int result = packetHandler->write2ByteTxRx(portHandler, dxlID, 30, goalPosition, &dxl_error);
    if (result != COMM_SUCCESS)
    {
        std::cerr << "Failed to write goal position: " << packetHandler->getTxRxResult(result) << std::endl;
        return false;
    }
    else if (dxl_error != 0)
    {
        std::cerr << "Error writing goal position: " << packetHandler->getRxPacketError(dxl_error) << std::endl;
        return false;
    }
    return true;
}