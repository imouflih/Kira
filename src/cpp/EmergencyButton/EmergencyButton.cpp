#include "EmergencyButton.hpp"

EmergencyButton::EmergencyButton() : driver(EmergencyButtonDriver()) {}

int EmergencyButton::isEmergencyButtonPressed() {
    std::cout << "Checking if Emergency Button is pressed" << std::endl;
    return this->driver.isEmergencyButtonPressed();
}

void EmergencyButton::stopTheRobot() {
    this->driver.sendStopOrderToArduino();
}