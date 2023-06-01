#include "EncoderWheelsDriver.hpp"
#include <iostream>

const uint8_t EncoderWheelsDriver::INIT_COUNTERS_MSG = 0xA1;    // Byte used to initialize the counters
const uint8_t EncoderWheelsDriver::START_BYTE = 0x25;           // Start byte to indicate the beginning of a message
const uint8_t EncoderWheelsDriver::STOP_BYTE = 0x5A;            // Stop byte to indicate the end of a message

EncoderWheelsDriver::EncoderWheelsDriver() : GenericDriver() {}

// Initialization of the counters
void EncoderWheelsDriver::initCounters() {
    this->writeData(&INIT_COUNTERS_MSG, sizeof(INIT_COUNTERS_MSG));
}

// Get counters from the arduino
std::pair<int, int> EncoderWheelsDriver::getCounters() {
    uint8_t data[10];

    // Reading the data from the arduino
    this->readData(data, 10);

    // Data structure : 
    // START_BYTE   LEFT_ENCODER    RIGHT_ENCODER   STOP_BYTE
    //   1 byte        4 bytes         4 bytes       1 byte

    // If the start byte or stop byte are incorrect, an exception is thrown
    if (data[0] != START_BYTE || data[9] != STOP_BYTE) {
        std::cout << "Erreur : data[0] : " << (uint8_t)data[0] << ", data[9] : " << (uint8_t)data[9] << std::endl;
        int countLeft = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24);
        int countRight = data[5] | (data[6] << 8) | (data[7] << 16) | (data[8] << 24);
        std::cout << "Error : counters : " << countLeft << "," << countRight << std::endl;
        throw "Failed to get counters! Invalid START_BYTE or STOP_BYTE";
    }

    // Extract the encoder count values from the data array
    int countLeft = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24);
    int countRight = data[5] | (data[6] << 8) | (data[7] << 16) | (data[8] << 24);

    // Return the encoder count values as a pair
    return std::make_pair(countLeft, countRight);
}
