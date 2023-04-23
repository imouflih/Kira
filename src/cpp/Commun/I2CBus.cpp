#include "I2CBus.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

I2CBus::I2CBus(const std::string& device, const int address_i2c) {
    fileDescriptor = open(device.c_str(), O_RDWR);
    if (fileDescriptor < 0) {
        throw "Failed to open i2c bus";
    }
    if (ioctl(fileDescriptor, I2C_SLAVE, address_i2c) < 0) {
        throw "Failed to acquire bus access or talk to slave";
    }
}

I2CBus::~I2CBus() {
    close(fileDescriptor);
}

I2CBus& I2CBus::getInstance(const std::string& device, const int address_i2c) {
    static I2CBus instance(device, address_i2c);
    return instance;
}

int I2CBus::getFile() {
    return fileDescriptor;
}
