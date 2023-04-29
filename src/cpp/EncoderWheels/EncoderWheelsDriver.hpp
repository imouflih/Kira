#include "../Commun/GenericDriver.hpp"

class EncoderWheelsDriver: GenericDriver {

public:
    EncoderWheelsDriver();
    void initCounters();
    std::tuple<int, int, long> getCounters();

private:
    static const uint8_t INIT_COUNTERS_MSG;
    static const uint8_t START_BYTE;
    static const uint8_t STOP_BYTE;

};
