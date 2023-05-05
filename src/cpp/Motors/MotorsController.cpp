#include "MotorsController.hpp"
#include <cmath>
#include <iostream>
#include <unistd.h>

const int MotorsController::MAX_SPEED = 120;
const int MotorsController::MIN_SPEED = 7;
const int MotorsController::MOUVEMENT_SPEED = 100;
const int MotorsController::ROTATION_SPEED = 15;
const float MotorsController::ROTATION_ANGLE_TOLERANCE = .02f;
const int MotorsController::MOUVEMENT_TOLERANCE = 20;

MotorsController::MotorsController() : driver(MotorsDriver()) {}

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

    // Calculate the angle difference
    float angleDifference = std::fmod(targetAngle - currentAngle + 8 * M_PI, 4 * M_PI);

    // Check if the angle difference is within the tolerance
    if (angleDifference < ROTATION_ANGLE_TOLERANCE || angleDifference >(4 * M_PI - ROTATION_ANGLE_TOLERANCE)) {
        return;
    }

    // Rotate the robot, make sure that it always takes the shortest path
    if (angleDifference < 2 * M_PI) {
        this->setMotorsSpeed(-ROTATION_SPEED, ROTATION_SPEED);
    }
    else {
        this->setMotorsSpeed(ROTATION_SPEED, -ROTATION_SPEED);
    }

    // Continue adjusting the angle until it is within the specified tolerance
    while ((std::abs(std::fmod(targetAngle - currentAngle + 8 * M_PI, 4 * M_PI))) > ROTATION_ANGLE_TOLERANCE) {
        currentAngle = getCurrentAngle();
        std::cout << "The error is : " << (std::abs(std::fmod(targetAngle - currentAngle + 8 * M_PI, 4 * M_PI))) << std::endl;
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

    float previousDistance = distance;
    int i = 0;
    sleep(0.3);
    doBeforeLinearMovement();

    // Speed ramp-up variables
    const int rampUpSteps = 10;
    const int speedIncrement = MOUVEMENT_SPEED / rampUpSteps;
    int currentSpeed = speedIncrement;

    // Move the robot forward towards the target position
    while (distance > MOUVEMENT_TOLERANCE) {
        int speedCorrection = getSpeedCorrection();

        // Ramp-up speed
        if (i < 2 * rampUpSteps) {
            this->setMotorsSpeed(currentSpeed - speedCorrection, currentSpeed + speedCorrection);
            currentSpeed += speedIncrement / 2.5;
        }
        else if (distance < 300) {
            this->setMotorsSpeed(currentSpeed - speedCorrection, currentSpeed + speedCorrection);
            currentSpeed -= speedIncrement;
            if (currentSpeed < 25) {
                currentSpeed = 25;
            }
        }
        else {
            this->setMotorsSpeed(MOUVEMENT_SPEED - speedCorrection, MOUVEMENT_SPEED + speedCorrection);
        }

        // Update the current position
        currentPosition = getCurrentPosition();

        // Calculate the new distance and angle to the target position
        dx = targetPosition.first - currentPosition.first;
        dy = targetPosition.second - currentPosition.second;
        distance = sqrt(dx * dx + dy * dy);

        std::cout << "ditance : " << distance << ", previousDistance : " << previousDistance << std::endl;
        if (distance > previousDistance && i > 1) {
            this->setMotorsSpeed(0, 0);
            std::cout << "The robot depaced the point, ditance : " << distance << ", previousDistance : " << previousDistance << std::endl;
            break;
        }

        previousDistance = distance;
        i++;
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