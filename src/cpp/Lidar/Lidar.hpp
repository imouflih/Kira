#include <iostream>
#include <unistd.h>
#include <vector>
#include <cmath>
#include <fstream>
#include "../../../../rplidar_sdk/rplidar_sdk/sdk/include/rplidar.h"

using namespace sl;

class Lidar
{
public:
    Lidar();
    ~Lidar();

    void startScanning();
    void stopLidar();
    void checkIfObstacleIsClose();

private:
    bool checkLidarResult();

    static const char* PORT;
    static const unsigned int BAUDRATE;
    static const int SPEED;
    static const int DANGER_DISTANCE;

    IChannel* channel;
    ILidarDriver* lidar;

    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t nodeCount = sizeof(nodes) / sizeof(sl_lidar_response_measurement_node_hq_t);
};
