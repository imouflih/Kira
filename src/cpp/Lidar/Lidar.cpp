#include "Lidar.hpp"

const char* Lidar::PORT = "/dev/ttyUSB1";
const unsigned int Lidar::BAUDRATE = 115200;
const int Lidar::SPEED = 512;
const int Lidar::DANGER_DISTANCE = 50;

Lidar::Lidar() {
    channel = *createSerialPortChannel(PORT, BAUDRATE);
    lidar = *createLidarDriver();

    auto res = lidar->connect(channel);
    if (!SL_IS_OK(res)) {
        std::cout << "Failed to connect to LIDAR : " << res << std::endl;
    }
}

Lidar::~Lidar() {
    stopLidar();
    delete lidar;
    delete channel;
}

void Lidar::startScanning() {
    lidar->setMotorSpeed(SPEED);

    std::vector<LidarScanMode> scanModes;
    lidar->getAllSupportedScanModes(scanModes);
    lidar->startScanExpress(false, scanModes[0].id);

    std::cout << "Scanning" << std::endl;
}

void Lidar::stopLidar() {
    lidar->stop();
}

void Lidar::checkIfObstacleIsClose() {
    while (true) {
        lidar->grabScanDataHq(nodes, nodeCount);
        lidar->ascendScanData(nodes, nodeCount);

        if (checkLidarResult()) {
            std::cout << "Warning!!!" << std::endl;
        }

        usleep(300000);
    }
}

bool Lidar::checkLidarResult() {
    int consecutivePoints = 0;

    for (size_t i = 0; i < nodeCount; ++i) {
        float distance = nodes[i].dist_mm_q2 / 10.f / (1 << 2);
        // float angle = nodes[i].angle_z_q14 * 90.f / (1 << 14);
        if (distance > 0 && distance < DANGER_DISTANCE) {
            consecutivePoints++;
            if (consecutivePoints >= 3) {
                return 1;
            }
        }
        else {
            consecutivePoints = 0;
        }
    }
    return 0;
}
