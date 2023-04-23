#pragma once

#include <cstdint>
#include <string>

class I2CBus;

class GenericDriver {
public:
    GenericDriver(const std::string deviceAddress = "/dev/i2c-1", const int address_i2c = 12);
    virtual ~GenericDriver() = default;

    // virtual bool init() = 0;

protected:
    I2CBus& i2cBus;

    void readData(uint8_t* data, uint16_t length);
    void writeData(const uint8_t* data, uint16_t length);
};
