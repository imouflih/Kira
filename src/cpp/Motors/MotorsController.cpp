#include "MotorsController.hpp"
#include <cmath>
#include <iostream>
#include <unistd.h>
#include <chrono>

const int MotorsController::MAX_SPEED = 120;
const int MotorsController::MIN_SPEED = 7;
const int MotorsController::MOUVEMENT_SPEED = 100;
const int MotorsController::ROTATION_SPEED = 15;
const float MotorsController::ROTATION_ANGLE_TOLERANCE = .02f;
const int MotorsController::MOUVEMENT_TOLERANCE = 25;

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
    float angleDifference = std::fmod(targetAngle - currentAngle + 4 * M_PI, 2 * M_PI);

    // Check if the angle difference is within the tolerance
    if (angleDifference < ROTATION_ANGLE_TOLERANCE || angleDifference >(2 * M_PI - ROTATION_ANGLE_TOLERANCE)) {
        return;
    }

    // Rotate the robot, make sure that it always takes the shortest path
    if (angleDifference < M_PI) {
        this->setMotorsSpeed(-ROTATION_SPEED, ROTATION_SPEED);
    }
    else {
        this->setMotorsSpeed(ROTATION_SPEED, -ROTATION_SPEED);
    }

    // Continue adjusting the angle until it is within the specified tolerance
    while ((std::abs(std::fmod(targetAngle - currentAngle + 4 * M_PI, 2 * M_PI))) > ROTATION_ANGLE_TOLERANCE) {
        currentAngle = getCurrentAngle();
        std::cout << "The error is : " << (std::abs(std::fmod(targetAngle - currentAngle + 4 * M_PI, 2 * M_PI))) << std::endl;
    }

    this->setMotorsSpeed(0, 0); // Stop the robot
}

void MotorsController::goToPosition(
    std::pair<float, float> targetPosition,
    std::function<std::pair<float, float>()> getCurrentPosition,
    std::function<float()> getCurrentAngle,
    std::function<int()> getSpeedCorrection,
    std::function<void()> doBeforeLinearMovement,
    std::function<bool()> obstacleIsClose,
    int mouvementSpeed) {

    std::pair<float, float> currentPosition = getCurrentPosition();

    // Calculate the distance between the current position and target position
    float dx = targetPosition.first - currentPosition.first;
    float dy = targetPosition.second - currentPosition.second;
    float distance = sqrt(dx * dx + dy * dy);
    float dTheta = atan2(dy, dx);
    dTheta = fmod(dTheta, 2 * M_PI) < 0 ? fmod(dTheta, 2 * M_PI) + 4 * M_PI : fmod(dTheta, 2 * M_PI);

    this->rotate(dTheta, getCurrentAngle);

    float previousDistance = distance;
    int i = 0;
    int j = 0;
    doBeforeLinearMovement();

    // Speed ramp-up variables
    const int rampUpSteps = 10;
    const int speedIncrement = mouvementSpeed / rampUpSteps;
    int currentSpeed = speedIncrement;

    // Move the robot forward towards the target position
    while (distance > MOUVEMENT_TOLERANCE) {
        if (obstacleIsClose()) {
            this->setMotorsSpeed(0, 0);
            sleep(3);
            continue;
        }

        int speedCorrection = getSpeedCorrection();

        // Ramp-up speed
        if (i < 2 * rampUpSteps) {
            this->setMotorsSpeed(currentSpeed - speedCorrection, currentSpeed + speedCorrection);
            currentSpeed += speedIncrement / 3.5;
        }
        else if (distance < 300) {
            this->setMotorsSpeed(currentSpeed - speedCorrection, currentSpeed + speedCorrection);
            currentSpeed -= speedIncrement;
            if (currentSpeed < 35) {
                currentSpeed = 35;
            }
        }
        else {
            this->setMotorsSpeed(mouvementSpeed - speedCorrection, mouvementSpeed + speedCorrection);
        }

        // Update the current position
        currentPosition = getCurrentPosition();

        // Calculate the new distance and angle to the target position
        dx = targetPosition.first - currentPosition.first;
        dy = targetPosition.second - currentPosition.second;
        distance = sqrt(dx * dx + dy * dy);

        std::cout << "distance : " << distance << ", previousDistance : " << previousDistance << std::endl;
        if (distance > previousDistance + 1 && i > 1) {
            this->setMotorsSpeed(0, 0);
            std::cout << "The robot depaced the point, distance : " << distance << ", previousDistance : " << previousDistance << std::endl;
            break;
        }

        previousDistance = distance;
        i++;
        j++;

        // float currentAngle = getCurrentAngle();
        // float diff = std::fmod((currentAngle - dTheta + 3 * M_PI), (2 * M_PI)) - M_PI;

        // if (((diff > 1) || (diff < -1)) && j > 30) {
        //     this->setMotorsSpeed(0, 0);
        //     usleep(500000);
        //     std::cout << "The robot went out of his trajectory" << std::endl;
        //     this->rotate(dTheta, getCurrentAngle);
        //     usleep(100000);
        //     doBeforeLinearMovement();
        // }
    }

    this->setMotorsSpeed(0, 0);
}

void MotorsController::goToPositionBackward(
    std::pair<float, float> targetPosition,
    std::function<std::pair<float, float>()> getCurrentPosition,
    std::function<float()> getCurrentAngle,
    std::function<int()> getSpeedCorrection,
    std::function<void()> doBeforeLinearMovement,
    int mouvementSpeed) {

    std::pair<float, float> currentPosition = getCurrentPosition();

    // Calculate the distance between the current position and target position
    float dx = targetPosition.first - currentPosition.first;
    float dy = targetPosition.second - currentPosition.second;
    float distance = sqrt(dx * dx + dy * dy);
    float dTheta = atan2(dy, dx) + M_PI;
    dTheta = fmod(dTheta, 2 * M_PI) < 0 ? fmod(dTheta, 2 * M_PI) + 2 * M_PI : fmod(dTheta, 2 * M_PI);

    this->rotate(dTheta, getCurrentAngle);

    float previousDistance = distance;
    int i = 0;
    int j = 0;
    usleep(300000);
    doBeforeLinearMovement();

    // Speed ramp-up variables
    const int rampUpSteps = 10;
    const int speedIncrement = mouvementSpeed / rampUpSteps;
    int currentSpeed = speedIncrement;

    // Move the robot backward towards the target position
    while (distance > MOUVEMENT_TOLERANCE) {
        int speedCorrection = getSpeedCorrection();

        // Ramp-up speed
        if (i < 2 * rampUpSteps) {
            this->setMotorsSpeed(-currentSpeed - speedCorrection, -currentSpeed + speedCorrection);
            currentSpeed += speedIncrement / 3.5;
        }
        else if (distance < 300) {
            this->setMotorsSpeed(-currentSpeed - speedCorrection, -currentSpeed + speedCorrection);
            currentSpeed -= speedIncrement;
            if (currentSpeed < 25) {
                currentSpeed = 25;
            }
        }
        else {
            this->setMotorsSpeed(-mouvementSpeed - speedCorrection, -mouvementSpeed + speedCorrection);
        }

        // Update the current position
        currentPosition = getCurrentPosition();

        // Calculate the new distance and angle to the target position
        dx = targetPosition.first - currentPosition.first;
        dy = targetPosition.second - currentPosition.second;
        distance = sqrt(dx * dx + dy * dy);

        std::cout << "distance : " << distance << ", previousDistance : " << previousDistance << std::endl;
        if (distance > previousDistance + 1 && i > 1) {
            this->setMotorsSpeed(0, 0);
            std::cout << "The robot depaced the point, distance : " << distance << ", previousDistance : " << previousDistance << std::endl;
            break;
        }

        previousDistance = distance;
        i++;
        j++;

        float currentAngle = getCurrentAngle();
        float diff = std::fmod((currentAngle - dTheta + 3 * M_PI), (2 * M_PI)) - M_PI;

        if (((diff > 1) || (diff < -1)) && j > 30) {
            this->setMotorsSpeed(0, 0);
            usleep(500000);
            std::cout << "The robot went out of his trajectory" << std::endl;
            this->rotate(dTheta + M_PI, getCurrentAngle);
            usleep(100000);
            doBeforeLinearMovement();
        }
    }

    this->setMotorsSpeed(0, 0);
}

void MotorsController::goForward(
    std::function<int()> getSpeedCorrection,
    std::function<void()> doBeforeLinearMovement,
    float duration) {

    doBeforeLinearMovement();

    // Speed ramp-up variables
    const int rampUpSteps = 10;
    const int speedIncrement = MOUVEMENT_SPEED / rampUpSteps;
    int currentSpeed = speedIncrement;
    int i = 0;

    // get start time
    auto start = std::chrono::system_clock::now();

    while (true) {
        // get current time
        auto now = std::chrono::system_clock::now();
        // calculate elapsed time in seconds
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();

        // if elapsed time is greater than or equal to the desired duration, break the loop
        if (elapsed >= duration) {
            break;
        }

        int speedCorrection = getSpeedCorrection();
        this->setMotorsSpeed(MOUVEMENT_SPEED - speedCorrection, MOUVEMENT_SPEED + speedCorrection);
        // Ramp-up speed
        if (i < 2 * rampUpSteps) {
            this->setMotorsSpeed(currentSpeed - speedCorrection, currentSpeed + speedCorrection);
            currentSpeed += speedIncrement / 3.5;
        }
        else {
            this->setMotorsSpeed(MOUVEMENT_SPEED - speedCorrection, MOUVEMENT_SPEED + speedCorrection);
        }

        i++;
    }

    // Stop the motors after moving forward for the specified duration
    this->setMotorsSpeed(0, 0);
}

void MotorsController::goBackward(
    std::function<int()> getSpeedCorrection,
    std::function<void()> doBeforeLinearMovement,
    float duration) {

    doBeforeLinearMovement();

    // Speed ramp-up variables
    const int rampUpSteps = 10;
    const int speedIncrement = MOUVEMENT_SPEED / rampUpSteps;
    int currentSpeed = speedIncrement;
    int i = 0;

    // get start time
    auto start = std::chrono::system_clock::now();

    while (true) {
        // get current time
        auto now = std::chrono::system_clock::now();
        // calculate elapsed time in seconds
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();

        // if elapsed time is greater than or equal to the desired duration, break the loop
        if (elapsed >= duration) {
            break;
        }

        int speedCorrection = getSpeedCorrection();
        this->setMotorsSpeed(-MOUVEMENT_SPEED - speedCorrection, -MOUVEMENT_SPEED + speedCorrection);
        // Ramp-up speed
        if (i < 2 * rampUpSteps) {
            this->setMotorsSpeed(-currentSpeed - speedCorrection, -currentSpeed + speedCorrection);
            currentSpeed += speedIncrement / 3.5;
        }
        else {
            this->setMotorsSpeed(-MOUVEMENT_SPEED - speedCorrection, -MOUVEMENT_SPEED + speedCorrection);
        }

        i++;
    }

    // Stop the motors after moving forward for the specified duration
    this->setMotorsSpeed(0, 0);
}