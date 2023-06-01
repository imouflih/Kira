#include "../Commun/GenericDriver.hpp"

// This class is used to insure communication of the encoder wheels values between the raspberry and the arduino (low-level control)
class EncoderWheelsDriver : GenericDriver {

public:
    EncoderWheelsDriver();
    void initCounters();

    // Returns a pair of integers, where the first element is the left counter value and the second is the right counter value
    std::pair<int, int> getCounters();

private:
    static const uint8_t INIT_COUNTERS_MSG;
    static const uint8_t START_BYTE;
    static const uint8_t STOP_BYTE;

};
