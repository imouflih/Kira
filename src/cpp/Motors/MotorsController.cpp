#include "MotorsController.hpp"
#include <cmath>

const int MotorsController::MAX_SPEED = 80;
const int MotorsController::MIN_SPEED = 7;
const int MotorsController::MOUVEMENT_SPEED = 30;
const int MotorsController::ROTATION_SPEED = 15;
const float MotorsController::ROTATION_ANGLE_TOLERANCE = .03f;
const int MotorsController::MOUVEMENT_TOLERANCE = 40;

MotorsController::MotorsController(): driver(MotorsDriver()) {}

void MotorsController::setMotorsSpeed(int speedLeft, int speedRight) {
    // make sure speed doesn't exceed the max and min speed
    int limitedSpeedRight =
        (speedRight != 0) ?
        ((speedRight > 0) ? 1 : -1) * std::max(MotorsController::MIN_SPEED, std::min(abs(speedRight), MotorsController::MAX_SPEED)) :
        0;
    int limitedSpeedLeft =
        (speedLeft != 0) ?
        ((speedLeft > 0) ? 1 : -1) * std::max(MotorsController::MIN_SPEED, std::min(abs(speedLeft), MotorsController::MAX_SPEED)) :
        0;

    this->driver.setMotorsSpeed(limitedSpeedLeft, limitedSpeedRight);
}

void MotorsController::rotate(float targetAngle, std::function<float()> getCurrentAngle) {
    // Get the current angle of the robot
    float currentAngle = getCurrentAngle();

    // Rotate the robot, make sure that he always takes the shortest path
    fabs(targetAngle - currentAngle) < 2 * M_PI ?
        targetAngle > currentAngle ?
        this->setMotorsSpeed(-ROTATION_SPEED, ROTATION_SPEED) :
        this->setMotorsSpeed(ROTATION_SPEED, -ROTATION_SPEED) :
        targetAngle > currentAngle ?
        this->setMotorsSpeed(ROTATION_SPEED, -ROTATION_SPEED) :
        this->setMotorsSpeed(-ROTATION_SPEED, ROTATION_SPEED);

    // Continue adjusting the angle until it is within the specified tolerance
    while (fmod(fabs(targetAngle - currentAngle), 4 * M_PI) > ROTATION_ANGLE_TOLERANCE) {

        // Update the current angle of the robot
        currentAngle = getCurrentAngle();
    }

    this->setMotorsSpeed(0, 0); // Stop the robot
}

void MotorsController::goToPosition(
    std::pair<float, float> targetPosition,
    std::function<std::pair<float, float>()> getCurrentPosition,
    std::function<float()> getCurrentAngle,
    std::function<int()> getSpeedCorrection,
    std::function<void()> doBeforeLinearMovement) {

    std::pair<float, float> currentPosition = getCurrentPosition();

    // Calculate the distance between the current position and target position
    float dx = targetPosition.first - currentPosition.first;
    float dy = targetPosition.second - currentPosition.second;
    float distance = sqrt(dx * dx + dy * dy);
    float dTheta = atan2(dy, dx);
    dTheta = fmod(dTheta, 4 * M_PI) < 0 ? fmod(dTheta, 4 * M_PI) + 4 * M_PI : fmod(dTheta, 4 * M_PI);

    this->rotate(dTheta, getCurrentAngle);
    doBeforeLinearMovement();
    // Move the robot forward towards the target position
    while (distance > MOUVEMENT_TOLERANCE) {
        int speedCorrection = getSpeedCorrection();
        this->setMotorsSpeed(MOUVEMENT_SPEED - speedCorrection, MOUVEMENT_SPEED + speedCorrection);
        // Update the current position
        currentPosition = getCurrentPosition();

        // Calculate the new distance and angle to the target position
        dx = targetPosition.first - currentPosition.first;
        dy = targetPosition.second - currentPosition.second;
        distance = sqrt(dx * dx + dy * dy);
    }

    this->setMotorsSpeed(0, 0);
}

void MotorsController::goForward(
    std::function<int()> getSpeedCorrection,
    std::function<void()> doBeforeLinearMovement) {
    doBeforeLinearMovement();
    while (true) {
        int speedCorrection = getSpeedCorrection();
        this->setMotorsSpeed(MOUVEMENT_SPEED - speedCorrection, MOUVEMENT_SPEED + speedCorrection);
    }
}

void MotorsController::goBackward(
    std::function<int()> getSpeedCorrection,
    std::function<void()> doBeforeLinearMovement) {
    doBeforeLinearMovement();
    while (true) {
        int speedCorrection = getSpeedCorrection();
        this->setMotorsSpeed(-MOUVEMENT_SPEED - speedCorrection, -MOUVEMENT_SPEED + speedCorrection);
    }
}