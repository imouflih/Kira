#include "MotorsDriver.hpp"
#include <iostream>

MotorsDriver::MotorsDriver() : GenericDriver() {}

// Function to set the speed of the motors
void MotorsDriver::setMotorsSpeed(int speedRight, int speedLeft) {
    std::cout << "setMotorsSpeed to driver : " << speedLeft << "," << speedRight << std::endl;
    uint8_t data[4];

    // Convert the speeds into 16-bit format for transmission
    data[0] = speedLeft & 0xFF;
    data[1] = (speedLeft >> 8) & 0xFF;
    data[2] = speedRight & 0xFF;
    data[3] = (speedRight >> 8) & 0xFF;

    this->writeData(data, 4);
}
