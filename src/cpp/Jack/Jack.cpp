#include "Jack.hpp"

const int Jack::JACK_PIN = 23;

Jack::Jack() {
    if (wiringPiSetup() == -1) {
        std::cout << "Erreur d'initialisation de wiringPi!" << std::endl;
        exit(-1);
    }
    pinMode(JACK_PIN, INPUT);
    pullUpDnControl(JACK_PIN, PUD_UP);
}

int Jack::isJackRemoved() {
    std::cout << "Checking if Jack is removed" << std::endl;
    return digitalRead(JACK_PIN);
}