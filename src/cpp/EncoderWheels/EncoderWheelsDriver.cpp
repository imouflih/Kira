#include "EncoderWheelsDriver.hpp"
#include <iostream>

const uint8_t EncoderWheelsDriver::INIT_COUNTERS_MSG = 0xA1;
const uint8_t EncoderWheelsDriver::START_BYTE = 0x25;
const uint8_t EncoderWheelsDriver::STOP_BYTE = 0x5A;

EncoderWheelsDriver::EncoderWheelsDriver(): GenericDriver() {}

void EncoderWheelsDriver::initCounters() {
    this->writeData(&INIT_COUNTERS_MSG, sizeof(INIT_COUNTERS_MSG));
}

std::pair<int, int> EncoderWheelsDriver::getCounters() {
    uint8_t data[10];
    this->readData(data, 10);

    if (data[0] != START_BYTE || data[9] != STOP_BYTE) {
        std::cout << "Erreur : data[0] : " << (uint8_t)data[0] << ", data[9] : " << (uint8_t)data[9] << std::endl;
        int countLeft = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24);
        int countRight = data[5] | (data[6] << 8) | (data[7] << 16) | (data[8] << 24);
        std::cout << "Error : counters : " << countLeft << "," << countRight << std::endl;
        throw "Failed to get counters! Invalid START_BYTE or STOP_BYTE";
    }

    int countLeft = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24);
    int countRight = data[5] | (data[6] << 8) | (data[7] << 16) | (data[8] << 24);

    return std::make_pair(countLeft, countRight);
}
