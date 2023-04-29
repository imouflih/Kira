#include "EncoderWheelsDriver.hpp"
#include <iostream>
#include <tuple>

const uint8_t EncoderWheelsDriver::INIT_COUNTERS_MSG = 0xA1;
const uint8_t EncoderWheelsDriver::START_BYTE = 0x25;
const uint8_t EncoderWheelsDriver::STOP_BYTE = 0x5A;

EncoderWheelsDriver::EncoderWheelsDriver(): GenericDriver() {}

void EncoderWheelsDriver::initCounters() {
    this->writeData(&INIT_COUNTERS_MSG, sizeof(INIT_COUNTERS_MSG));
}

std::tuple<int, int, long> EncoderWheelsDriver::getCounters() {
    uint8_t data[14];
    this->readData(data, 14);

    if (data[0] != START_BYTE || data[13] != STOP_BYTE) {
        std::cout << "Erreur : data[0] : " << (uint8_t)data[0] << ", data[13] : " << (uint8_t)data[13] << std::endl;
        int countLeft = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24);
        int countRight = data[5] | (data[6] << 8) | (data[7] << 16) | (data[8] << 24);
        long elapsedTime = data[9] | (data[10] << 8) | (data[11] << 16) | (data[12] << 24);
        std::cout << "Error : counters : " << countLeft << "," << countRight << std::endl;
        std::cout << "Error : Elapsed time : " << elapsedTime << std::endl;
        throw "Failed to get counters! Invalid START_BYTE or STOP_BYTE";
    }

    int countLeft = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24);
    int countRight = data[5] | (data[6] << 8) | (data[7] << 16) | (data[8] << 24);
    long elapsedTime = data[9] | (data[10] << 8) | (data[11] << 16) | (data[12] << 24);

    return std::make_tuple(countLeft, countRight, elapsedTime);
}
