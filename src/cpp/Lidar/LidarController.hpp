#include <iostream>
#include <unistd.h>
#include <vector>
#include <cmath>
#include <fstream>
#include "../../../../rplidar_sdk/rplidar_sdk/sdk/include/rplidar.h"
#include "../Commun/Time/Tempo.hpp"

// This class provides an interface for controlling and obtaining data from a LIDAR, it uses RPLIDAR SDK
class LidarController
{
public:
    LidarController();
    ~LidarController();

    void init();
    bool checkIfObstacleIsClose(int side);

private:
    // Constants defining LIDAR communication, operation speed, and obstacle detection settings
    static const char* PORT;
    static const unsigned int BAUDRATE;
    static const int SPEED;
    static const int DANGER_DISTANCE;

    sl::IChannel* channel;
    sl::ILidarDriver* driver;
    Tempo tempo;

    // Array for storing the data nodes from the LIDAR
    sl_lidar_response_measurement_node_hq_t nodes[8192];

    // The number of nodes that are currently stored
    size_t nodeCount = sizeof(nodes) / sizeof(sl_lidar_response_measurement_node_hq_t);

    // Stores the result of the last obstacle check
    bool lastCheckResult;

    bool checkLidarResult();
    bool checkLidarResultForward();
    bool checkLidarResultBackward();
    void startLidar();
    void stopLidar();
};
