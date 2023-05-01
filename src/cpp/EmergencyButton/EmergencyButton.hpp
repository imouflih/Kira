#include <wiringPi.h>
#include <iostream>

class EmergencyButton {
public:
    EmergencyButton();
    int isEmergencyButtonPressed();

private:
    static const int EMERGENCY_BUTTON_PIN;
};