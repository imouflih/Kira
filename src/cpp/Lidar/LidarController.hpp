#include <iostream>
#include <unistd.h>
#include <vector>
#include <cmath>
#include <fstream>
#include "../../../../rplidar_sdk/rplidar_sdk/sdk/include/rplidar.h"
#include "../Commun/Time/Tempo.hpp"

class LidarController
{
public:
    LidarController();
    ~LidarController();

    void init();
    bool checkIfObstacleIsClose(int side);

private:
    static const char* PORT;
    static const unsigned int BAUDRATE;
    static const int SPEED;
    static const int DANGER_DISTANCE;

    sl::IChannel* channel;
    sl::ILidarDriver* driver;
    Tempo tempo;

    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t nodeCount = sizeof(nodes) / sizeof(sl_lidar_response_measurement_node_hq_t);
    bool lastCheckResult;

    bool checkLidarResult();
    bool checkLidarResultForward();
    bool checkLidarResultBackward();
    void startLidar();
    void stopLidar();
};
