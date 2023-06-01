#include "DisguiseLED.hpp"

const int DisguiseLED::DISGUISE_LED_PIN = 11;

DisguiseLED::DisguiseLED() {
    if (wiringPiSetup() == -1) {
        std::cout << "Erreur d'initialisation de wiringPi!" << std::endl;
        exit(-1);
    }
    pinMode(DISGUISE_LED_PIN, OUTPUT);
}

void DisguiseLED::TurnOn() {
    std::cout << "Turning the Disguise LED on" << std::endl;
    digitalWrite(DISGUISE_LED_PIN, 1);
}

void DisguiseLED::TurnOff() {
    std::cout << "Turning the Disguise LED off" << std::endl;
    digitalWrite(DISGUISE_LED_PIN, 0);
}