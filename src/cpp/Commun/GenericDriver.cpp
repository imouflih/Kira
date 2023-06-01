#include "GenericDriver.hpp"
#include <unistd.h>
#include <I2CBus.hpp>

GenericDriver::GenericDriver(const std::string deviceAddress, const int address_i2c) :i2cBus(I2CBus::getInstance(deviceAddress, address_i2c)) {}

// Function to read data from the arduino
void GenericDriver::readData(uint8_t* data, uint16_t length) {
    if (read(this->i2cBus.getFile(), data, length) != length) {
        throw "Failed to read data from i2c bus.";
    }
}

// Function to write data to the arduino
void GenericDriver::writeData(const uint8_t* data, uint16_t length) {
    if (write(this->i2cBus.getFile(), data, length) == -1) {
        throw "Failed to write to I2C bus.";
    }
}
