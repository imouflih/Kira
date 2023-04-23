#include "MotorsDriver.hpp"
#include <iostream>

MotorsDriver::MotorsDriver(): GenericDriver() {}

void MotorsDriver::setMotorsSpeed(int speedRight, int speedLeft) {
    std::cout << "setMotorsSpeed to driver : " << speedLeft << "," << speedRight << std::endl;
    uint8_t data[4];
    data[0] = speedLeft & 0xFF;
    data[1] = (speedLeft >> 8) & 0xFF;
    data[2] = speedRight & 0xFF;
    data[3] = (speedRight >> 8) & 0xFF;

    this->writeData(data, 4);
}
