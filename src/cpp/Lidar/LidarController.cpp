#include "LidarController.hpp"

// Constants
const char* LidarController::PORT = "/dev/ttyUSB0";
const unsigned int LidarController::BAUDRATE = 115200;
const int LidarController::SPEED = 512;
const int LidarController::DANGER_DISTANCE = 50;

LidarController::LidarController() {
    this->channel = *sl::createSerialPortChannel(PORT, BAUDRATE);
    this->driver = *sl::createLidarDriver();
    this->tempo = Tempo();

    this->lastCheckResult = false;

    auto res = this->driver->connect(this->channel);
    if (!SL_IS_OK(res)) {
        std::cout << "Failed to connect to LIDAR : " << res << std::endl;
    }
}

LidarController::~LidarController() {
    stopLidar();
    delete this->driver;
    delete this->channel;
}

// Initializes the LIDAR by starting it
void LidarController::init() {
    this->startLidar();
}

// Starts the LIDAR : setting speed and starting scanning
void LidarController::startLidar() {
    this->driver->setMotorSpeed(SPEED);

    std::vector<sl::LidarScanMode> scanModes;
    this->driver->getAllSupportedScanModes(scanModes);
    this->driver->startScanExpress(true, scanModes[0].id);

    std::cout << "Scanning" << std::endl;
}

// Stops the LIDAR
void LidarController::stopLidar() {
    this->driver->stop();
    // if the lidar keep return error in the end of main program, comment the line before and uncomment this line
    // this->driver->setMotorSpeed(0);
}

// Checks if an obstacle is close, based on the given side (0 for all directions, 1 for forward only, 2 for backward)
bool LidarController::checkIfObstacleIsClose(int side) {
    if (!this->tempo.hasPassed()) {
        std::cout << "Tempo has not passed yet" << std::endl;
        return this->lastCheckResult;
    }
    this->driver->grabScanDataHq(nodes, nodeCount);
    this->driver->ascendScanData(nodes, nodeCount);

    bool checkLidarResult;

    // 0 for all direction, 1 for forward only, 2 for backward
    switch (side) {
    case 0:
        checkLidarResult = this->checkLidarResult();
        break;
    case 1:
        checkLidarResult = this->checkLidarResultForward();
        break;
    case 2:
        checkLidarResult = this->checkLidarResultBackward();
        break;
    }
    if (checkLidarResult) {
        this->lastCheckResult = true;
    }
    else {
        this->lastCheckResult = false;
        std::cout << "There's no obstacle, the robot can keep going" << std::endl;
    }

    this->tempo.store();
    return lastCheckResult;
}

// Checks if there's any obstacle in all directions within the danger distance
bool LidarController::checkLidarResult() {
    int consecutivePoints = 0;

    for (size_t i = 0; i < nodeCount; ++i) {
        float distance = nodes[i].dist_mm_q2 / 10.f / (1 << 2);
        // float angle = nodes[i].angle_z_q14 * 90.f / (1 << 14);
        if (distance > 0 && distance < DANGER_DISTANCE) {
            consecutivePoints++;
            if (consecutivePoints >= 3) {
                std::cout << "Close Obstacle, distance : " << distance << std::endl;
                return 1;
            }
        }
        else {
            consecutivePoints = 0;
        }
    }
    return 0;
}

// Checks if there's any obstacle in the forward direction within the danger distance
bool LidarController::checkLidarResultForward() {
    int consecutivePoints = 0;

    for (size_t i = 0; i < nodeCount; ++i) {
        float distance = nodes[i].dist_mm_q2 / 10.f / (1 << 2);
        float angle = nodes[i].angle_z_q14 * 90.f / (1 << 14);
        if ((angle >= 235 || angle <= 55)) {
            if (distance > 0 && distance < DANGER_DISTANCE) {
                consecutivePoints++;
                if (consecutivePoints >= 3) {
                    std::cout << "Close Obstacle, distance : " << distance << std::endl;
                    return 1;
                }
            }
            else {
                consecutivePoints = 0;
            }
        }
    }
    return 0;
}

// Checks if there's any obstacle in the backward direction within the danger distance
bool LidarController::checkLidarResultBackward()
{
    int consecutivePoints = 0;

    for (size_t i = 0; i < nodeCount; ++i) {
        float distance = nodes[i].dist_mm_q2 / 10.f / (1 << 2);
        float angle = nodes[i].angle_z_q14 * 90.f / (1 << 14);
        if (angle >= 55 && angle <= 235) {
            if (distance > 0 && distance < DANGER_DISTANCE) {
                consecutivePoints++;
                if (consecutivePoints >= 3) {
                    std::cout << "Close Obstacle, distance : " << distance << std::endl;
                    return 1;
                }
            }
            else {
                consecutivePoints = 0;
            }
        }
    }
    return 0;
}

//! Note : the last 3 functions can be regrouped in the same function by adding a variable in order to improve the code
