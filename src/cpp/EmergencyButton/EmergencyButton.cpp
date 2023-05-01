#include "EmergencyButton.hpp"

const int EmergencyButton::EMERGENCY_BUTTON_PIN = 24;

EmergencyButton::EmergencyButton() {
    if (wiringPiSetup() == -1) {
        std::cout << "Erreur d'initialisation de wiringPi!" << std::endl;
        exit(-1);
    }
    pinMode(EMERGENCY_BUTTON_PIN, INPUT);
}

int EmergencyButton::isEmergencyButtonPressed() {
    std::cout << "Checking if Emergency Button is pressed" << std::endl;
    return !digitalRead(EMERGENCY_BUTTON_PIN);
}