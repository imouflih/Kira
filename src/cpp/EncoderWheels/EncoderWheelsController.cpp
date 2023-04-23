#include "EncoderWheelsController.hpp"
#include <cmath>
#include <iostream>

const unsigned short EncoderWheelsController::ENTRAXE = 235;
const unsigned short EncoderWheelsController::DIAMETER = 35;
const unsigned short EncoderWheelsController::TICK_PER_TURN = 512;
const float EncoderWheelsController::PROPORTIONAL_GAIN = .1f;

EncoderWheelsController::EncoderWheelsController(Coordinates initialCoordinates):
    driver(EncoderWheelsDriver()),
    previousCounters(std::make_pair(0, 0)),
    coordinates(initialCoordinates) {}

EncoderWheelsController::Coordinates::Coordinates(std::pair<float, float> position, float orientationAngle):
    position(position),
    orientationAngle(orientationAngle) {}

void EncoderWheelsController::initCounters() {
    this->driver.initCounters();
}

EncoderWheelsController::Coordinates EncoderWheelsController::updateAndGetCoordinates() {
    std::pair<int, int> counters = this->driver.getCounters();

    std::cout << "Counters : (" << counters.first << "," << counters.second << ")" << std::endl;

    float distanceLeft = calculateWheelDistance(counters.first, this->previousCounters.first);
    float distanceRight = calculateWheelDistance(counters.second, this->previousCounters.second);

    float distanceAverage = calculateAverageDistance(distanceLeft, distanceRight);
    float angle = calculateAngleTurned(distanceLeft, distanceRight);

    updateCoordinates(distanceAverage, angle);

    this->previousCounters = counters;

    return this->coordinates;
}

float EncoderWheelsController::calculateWheelDistance(int currentCounter, int previousCounter) const {
    return (currentCounter - previousCounter) * 2 * M_PI * DIAMETER / TICK_PER_TURN;
}

float EncoderWheelsController::calculateAverageDistance(float distanceLeft, float distanceRight) const {
    return (distanceLeft + distanceRight) / 2;
}

float EncoderWheelsController::calculateAngleTurned(float distanceLeft, float distanceRight) const {
    return (distanceRight - distanceLeft) / ENTRAXE;
}

void EncoderWheelsController::updateCoordinates(float distanceAverage, float angle) {
    float delta_x = distanceAverage * cos(this->coordinates.orientationAngle + angle / 2);
    float delta_y = distanceAverage * sin(this->coordinates.orientationAngle + angle / 2);
    float delta_theta = angle;

    this->coordinates.position.first += delta_x;
    this->coordinates.position.second += delta_y;
    this->coordinates.orientationAngle += delta_theta;

    this->coordinates.orientationAngle = normalizeOrientationAngle(this->coordinates.orientationAngle);
}

float EncoderWheelsController::normalizeOrientationAngle(float angle) const {
    float normalizedAngle = fmod(angle, 4 * M_PI);
    return (normalizedAngle < 0) ? (normalizedAngle + 4 * M_PI) : normalizedAngle;
}

int EncoderWheelsController::getCountersDifference() {
    std::pair<int, int> counters = this->driver.getCounters();
    return counters.first - counters.second;
}

int EncoderWheelsController::computeWheelsCorrection(int differenceCounts) {
    std::pair<int, int> counters = this->driver.getCounters();

    // Calculate the error between left and right encoder counts
    int error = counters.first - counters.second - differenceCounts;

    // Calculate and return the speed correction using proportional gain
    return static_cast<int>(round(PROPORTIONAL_GAIN * error));
}
