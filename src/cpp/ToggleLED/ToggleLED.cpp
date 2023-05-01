#include "ToggleLED.hpp"

const int ToggleLED::TOGGLE_LED_PIN = 25;

ToggleLED::ToggleLED() {
    if (wiringPiSetup() == -1) {
        std::cout << "Erreur d'initialisation de wiringPi!" << std::endl;
        exit(-1);
    }
    pinMode(TOGGLE_LED_PIN, INPUT);
}

void ToggleLED::TurnOn() {
    std::cout << "Turning the Toggle LED on" << std::endl;
    digitalWrite(TOGGLE_LED_PIN, 1);
}

void ToggleLED::TurnOff() {
    std::cout << "Turning the Toggle LED off" << std::endl;
    digitalWrite(TOGGLE_LED_PIN, 0);
}