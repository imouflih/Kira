#include "EmergencyButtonDriver.hpp"

#include <wiringPi.h>
#include <iostream>

class EmergencyButton {
public:
    EmergencyButton();
    int isEmergencyButtonPressed();
    void stopTheRobot();

private:
    EmergencyButtonDriver driver;
};