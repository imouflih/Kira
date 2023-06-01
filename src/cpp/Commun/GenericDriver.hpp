#pragma once

#include <cstdint>
#include <string>

class I2CBus;

// GenericDriver class provides operations (read/write) on an I2C bus
class GenericDriver {
public:
    // Defaults to "/dev/i2c-1" and 12 if no arguments are provided
    GenericDriver(const std::string deviceAddress = "/dev/i2c-1", const int address_i2c = 12);
    virtual ~GenericDriver() = default;

protected:
    I2CBus& i2cBus;

    void readData(uint8_t* data, uint16_t length);
    void writeData(const uint8_t* data, uint16_t length);
};
