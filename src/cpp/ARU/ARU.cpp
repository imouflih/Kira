#include "ARU.hpp"

const int ARU::ARU_PIN = 24;

ARU::ARU() {
    if (wiringPiSetup() == -1) {
        std::cout << "Erreur d'initialisation de wiringPi!" << std::endl;
        exit(-1);
    }
    pinMode(ARU_PIN, INPUT);
}

int ARU::isARUpressed() {
    std::cout << "Checking if ARU is pressed" << std::endl;
    return !digitalRead(ARU_PIN);
}