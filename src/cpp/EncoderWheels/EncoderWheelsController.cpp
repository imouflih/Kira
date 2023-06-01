#include "EncoderWheelsController.hpp"
#include <cmath>
#include <iostream>

// Constants
const unsigned short EncoderWheelsController::ENTRAXE = 280;        // Distance between two encoder wheels in mm
const unsigned short EncoderWheelsController::DIAMETER = 35;        // Diameter of the encoder wheels in mm
const unsigned short EncoderWheelsController::TICK_PER_TURN = 512;  // Number of ticks per a complete turn of the encoder wheels 
const float EncoderWheelsController::PROPORTIONAL_GAIN = .1f;       // Proportional gain

EncoderWheelsController::EncoderWheelsController(Coordinates initialCoordinates) :
    driver(EncoderWheelsDriver()),
    previousCounters(std::make_pair(0, 0)),
    coordinates(initialCoordinates) {}

EncoderWheelsController::Coordinates::Coordinates(std::pair<float, float> position, float orientationAngle) :
    position(position),
    orientationAngle(orientationAngle) {}

// Initialization of the counters
void EncoderWheelsController::init() {
    this->driver.initCounters();
}

// Update and return the current coordinates
EncoderWheelsController::Coordinates EncoderWheelsController::updateAndGetCoordinates() {
    std::pair<int, int> counters = this->driver.getCounters();

    std::cout << "Counters : (" << counters.first << "," << counters.second << ")" << std::endl;

    // Calculate the distance traveled by each wheel
    float distanceLeft = calculateWheelDistance(counters.first, this->previousCounters.first);
    float distanceRight = calculateWheelDistance(counters.second, this->previousCounters.second);

    // Calculate the average distance and the angle turned
    float distanceAverage = calculateAverageDistance(distanceLeft, distanceRight);
    float angle = calculateAngleTurned(distanceLeft, distanceRight);

    // Update the coordinates based on the calculated values
    updateCoordinates(distanceAverage, angle);

    this->previousCounters = counters;

    return this->coordinates;
}

// Calculate the distance a wheel has traveled based on the counter values
float EncoderWheelsController::calculateWheelDistance(int currentCounter, int previousCounter) const {
    return (currentCounter - previousCounter) * 2 * M_PI * DIAMETER / TICK_PER_TURN;
}

// Calculate the average distance traveled by the two wheels
float EncoderWheelsController::calculateAverageDistance(float distanceLeft, float distanceRight) const {
    return (distanceLeft + distanceRight) / 2;
}

// Calculate the angle turned based on the distance traveled by each wheel
float EncoderWheelsController::calculateAngleTurned(float distanceLeft, float distanceRight) const {
    return (distanceRight - distanceLeft) / ENTRAXE;
}

// Update the coordinates based on the average distance and the angle turned
void EncoderWheelsController::updateCoordinates(float distanceAverage, float angle) {
    // Calculate the change in position and orientation
    float delta_x = distanceAverage * cos(this->coordinates.orientationAngle + angle / 2);
    float delta_y = distanceAverage * sin(this->coordinates.orientationAngle + angle / 2);
    float delta_theta = angle;

    // Update the position and orientation
    this->coordinates.position.first += delta_x;
    this->coordinates.position.second += delta_y;
    this->coordinates.orientationAngle += delta_theta / 2;

    // Normalize the orientation angle
    this->coordinates.orientationAngle = normalizeOrientationAngle(this->coordinates.orientationAngle);
}

// Normalize the orientation angle to be within [0, 2*pi]
float EncoderWheelsController::normalizeOrientationAngle(float angle) const {
    float normalizedAngle = fmod(angle, 2 * M_PI);
    return (normalizedAngle < 0) ? (normalizedAngle + 2 * M_PI) : normalizedAngle;
}

// Get the difference between the counters of the two wheels
int EncoderWheelsController::getCountersDifference() {
    std::pair<int, int> counters = this->driver.getCounters();
    return counters.first - counters.second;
}

// Compute the speed correction for the wheels based on the difference in counts
int EncoderWheelsController::computeWheelsCorrection(int differenceCounts) {
    std::pair<int, int> counters = this->driver.getCounters();

    // Calculate the error between left and right encoder counts
    int error = counters.first - counters.second - differenceCounts;

    // Calculate and return the speed correction using proportional gain
    return static_cast<int>(round(PROPORTIONAL_GAIN * error));
}
