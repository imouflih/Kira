#include "../Commun/GenericDriver.hpp"

class EncoderWheelsDriver: GenericDriver {

public:
    EncoderWheelsDriver();
    void initCounters();
    std::pair<int, int> getCounters();

private:
    static const uint8_t INIT_COUNTERS_MSG;
    static const uint8_t START_BYTE;
    static const uint8_t STOP_BYTE;

};
