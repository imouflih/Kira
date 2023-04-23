#include <cstdint>
#include <string>

class I2CBus {
public:
    static I2CBus& getInstance(const std::string& device, const int address_i2c);
    I2CBus(const std::string& device, const int address_i2c);
    ~I2CBus();
    int getFile();

private:
    int fileDescriptor;
};
