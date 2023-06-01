#include "I2CBus.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

I2CBus::I2CBus(const std::string& device, const int address_i2c) {
    // Open the I2C device file
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

// Singleton accessor: Get a reference to the I2CBus instance for the specified 'device' and 'address_i2c'
// Note that this singleton doesn't allow to communicate with a second arduino
// If you need to connect to a second Arduino, consider revising this design by using a map of device/address pair or by removing singleton and and pay attention to multiple declaration
I2CBus& I2CBus::getInstance(const std::string& device, const int address_i2c) {
    static I2CBus instance(device, address_i2c);
    return instance;
}

// Get the file descriptor of the I2C device connection.
int I2CBus::getFile() {
    return fileDescriptor;
}
