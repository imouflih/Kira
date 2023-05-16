#include "../Motors/MotorsController.hpp"
#include "../EncoderWheels/EncoderWheelsController.hpp"
#include "../Lidar/LidarController.hpp"

class Coordinator {
public:
    Coordinator();
    void goToPosition(std::pair<int, int> targetPosition, int mouvementSpeed = MOUVEMENT_SPEED);
    void goToPositionBackward(std::pair<int, int> targetPosition, int mouvementSpeed = MOUVEMENT_SPEED);
    void init();
    void stop();
    void stopLidar();
    void rotate(float targetAngle);
    void goForward(float duration);
    void goBackward(float duration);

private:
    static const int MOUVEMENT_SPEED;

    MotorsController motorsController;
    EncoderWheelsController encoderWheelsController;
    LidarController lidarController;
    int initialCountersDifference;

    std::pair<float, float> getCurrentPosition();
    float getOrientation();
    int getSpeedCorrection();
    void updateInitialCountersDifference();
    bool obstacleIsClose(int side);
};
