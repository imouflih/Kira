#include <cstdint>
#include <string>

// I2CBus class represents a connection to an I2C device.
class I2CBus {
public:
    static I2CBus& getInstance(const std::string& device, const int address_i2c);
    I2CBus(const std::string& device, const int address_i2c);
    ~I2CBus();
    int getFile();

private:
    int fileDescriptor;
};
