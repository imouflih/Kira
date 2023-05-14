#include "../Commun/GenericDriver.hpp"

#include <wiringPi.h>
#include <iostream>

class EmergencyButtonDriver : GenericDriver {
public:
    EmergencyButtonDriver();
    void sendStopOrderToArduino();
    int isEmergencyButtonPressed();

private:
    static const int EMERGENCY_BUTTON_PIN;
    static const uint8_t EMERGENCY_STOP_MSG;
};