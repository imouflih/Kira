#include "EmergencyButtonDriver.hpp"

const int EmergencyButtonDriver::EMERGENCY_BUTTON_PIN = 24;
const uint8_t EmergencyButtonDriver::EMERGENCY_STOP_MSG = 0xA2;

EmergencyButtonDriver::EmergencyButtonDriver() : GenericDriver() {
    if (wiringPiSetup() == -1) {
        std::cout << "Erreur d'initialisation de wiringPi!" << std::endl;
        exit(-1);
    }
    pinMode(EMERGENCY_BUTTON_PIN, INPUT);
}

void EmergencyButtonDriver::sendStopOrderToArduino() {
    this->writeData(&EMERGENCY_STOP_MSG, sizeof(EMERGENCY_STOP_MSG));
    std::cout << "Emergency stop button is pressed, sending the order to the arduino" << std::endl;
}

int EmergencyButtonDriver::isEmergencyButtonPressed() {
    std::cout << "Checking if Emergency Button is pressed" << std::endl;
    return !digitalRead(EMERGENCY_BUTTON_PIN);
}