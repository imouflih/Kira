#include "Coordinator.hpp"
#include <iostream>

Coordinator::Coordinator():
    motorsController(MotorsController()),
    encoderWheelsController(EncoderWheelsController()),
    initialCountersDifference(0) {}

std::pair<float, float> Coordinator::getCurrentPosition() {
    std::pair<float, float> position = this->encoderWheelsController.updateAndGetCoordinates().position;
    std::cout << "getCurrentPosition : (" << position.first << "," << position.second << ")" << std::endl;
    return position;
}

float Coordinator::getOrientation() {
    float orientationAngle = this->encoderWheelsController.updateAndGetCoordinates().orientationAngle;
    std::cout << "getOrientation : " << orientationAngle << std::endl;
    return orientationAngle;

}

int Coordinator::getSpeedCorrection() {
    int speedCorrection = this->encoderWheelsController.computeWheelsCorrection(this->initialCountersDifference);
    std::cout << "getSpeedCorrection : " << speedCorrection << std::endl;
    return speedCorrection;
}

void Coordinator::updateInitialCountersDifference() {
    this->initialCountersDifference = encoderWheelsController.getCountersDifference();
    std::cout << "getInitialCountersDifference : " << initialCountersDifference << std::endl;
}

void Coordinator::goToPosition(std::pair<int, int> targetPosition) {

    std::cout << "GO TO POSITION : " << targetPosition.first << " , " << targetPosition.second << std::endl;
    auto lambdaGetCurrentPosition = [this]() { return this->getCurrentPosition(); };
    auto lambdaGetOrientation = [this]() { return this->getOrientation(); };
    auto lambdaGetSpeedCorrection = [this]() { return this->getSpeedCorrection(); };
    auto lambdadoBeforeLinearMovement = [this]() { this->updateInitialCountersDifference(); };

    this->motorsController.goToPosition(
        targetPosition,
        lambdaGetCurrentPosition,
        lambdaGetOrientation,
        lambdaGetSpeedCorrection,
        lambdadoBeforeLinearMovement);
}

void Coordinator::init() {
    std::cout << "Initialization" << std::endl;
    this->encoderWheelsController.initCounters();
}

void Coordinator::stop() {
    std::cout << "Stop" << std::endl;
    this->motorsController.setMotorsSpeed(0, 0);
}