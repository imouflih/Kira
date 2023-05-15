#include "Coordinator.hpp"
#include <iostream>

const int Coordinator::MOUVEMENT_SPEED = 100;

Coordinator::Coordinator() :
    motorsController(MotorsController()),
    encoderWheelsController(EncoderWheelsController()),
    lidarController(LidarController()),
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

bool Coordinator::obstacleIsClose() {
    bool obstacleIsClose = this->lidarController.checkIfObstacleIsClose();
    if (obstacleIsClose) {
        std::cout << "Warning!! An obstacle is close" << std::endl;
    }
    return obstacleIsClose;
}

void Coordinator::goToPosition(std::pair<int, int> targetPosition, int mouvementSpeed) {

    std::cout << "GO TO POSITION : " << targetPosition.first << " , " << targetPosition.second << std::endl;
    auto lambdaGetCurrentPosition = [this]() { return this->getCurrentPosition(); };
    auto lambdaGetOrientation = [this]() { return this->getOrientation(); };
    auto lambdaGetSpeedCorrection = [this]() { return this->getSpeedCorrection(); };
    auto lambdadoBeforeLinearMovement = [this]() { this->updateInitialCountersDifference(); };
    auto lambdadoObstacleIsClose = [this]() { return this->obstacleIsClose(); };

    this->motorsController.goToPosition(
        targetPosition,
        lambdaGetCurrentPosition,
        lambdaGetOrientation,
        lambdaGetSpeedCorrection,
        lambdadoBeforeLinearMovement,
        lambdadoObstacleIsClose,
        mouvementSpeed);
}

void Coordinator::goToPositionBackward(std::pair<int, int> targetPosition, int mouvementSpeed) {

    std::cout << "GO TO POSITION IN BACKWARD : " << targetPosition.first << " , " << targetPosition.second << std::endl;
    auto lambdaGetCurrentPosition = [this]() { return this->getCurrentPosition(); };
    auto lambdaGetOrientation = [this]() { return this->getOrientation(); };
    auto lambdaGetSpeedCorrection = [this]() { return this->getSpeedCorrection(); };
    auto lambdadoBeforeLinearMovement = [this]() { this->updateInitialCountersDifference(); };

    this->motorsController.goToPositionBackward(
        targetPosition,
        lambdaGetCurrentPosition,
        lambdaGetOrientation,
        lambdaGetSpeedCorrection,
        lambdadoBeforeLinearMovement,
        mouvementSpeed);
}

void Coordinator::init() {
    std::cout << "Initialization" << std::endl;
    this->encoderWheelsController.init();
    this->lidarController.init();
}

void Coordinator::stop() {
    std::cout << "Stop" << std::endl;
    this->motorsController.setMotorsSpeed(0, 0);
}
void Coordinator::stopLidar() {
    std::cout << "Stop Lidar" << std::endl;
    this->lidarController.~LidarController();
}

void Coordinator::rotate(float targetAngle) {
    std::cout << "Rotate to " << targetAngle << " rad" << std::endl;
    auto lambdaGetOrientation = [this]() { return this->getOrientation(); };
    this->motorsController.rotate(targetAngle, lambdaGetOrientation);
}

void Coordinator::goForward(float duration) {
    std::cout << "Go Forward for : " << duration << std::endl;
    auto lambdaGetSpeedCorrection = [this]() { return this->getSpeedCorrection(); };
    auto lambdadoBeforeLinearMovement = [this]() { this->updateInitialCountersDifference(); };
    this->motorsController.goForward(lambdaGetSpeedCorrection, lambdadoBeforeLinearMovement, duration);
}

void Coordinator::goBackward(float duration) {
    std::cout << "Go Backward for : " << duration << std::endl;
    auto lambdaGetSpeedCorrection = [this]() { return this->getSpeedCorrection(); };
    auto lambdadoBeforeLinearMovement = [this]() { this->updateInitialCountersDifference(); };
    this->motorsController.goBackward(lambdaGetSpeedCorrection, lambdadoBeforeLinearMovement, duration);
}