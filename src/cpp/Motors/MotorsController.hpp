#include "MotorsDriver.hpp"
#include <functional>

// The MotorsController class provides higher-level control over the motors
class MotorsController {
public:
    MotorsController();
    void setMotorsSpeed(int speedLeft, int speedRight);
    void rotate(float targetAngle,
        std::function<float()> getCurrentAngle,
        std::function<bool()> obstacleIsClose);
    void goToPosition(
        std::pair<float, float> targetPosition,
        std::function<std::pair<float, float>()> getCurrentPosition,
        std::function<float()> getCurrentAngle,
        std::function<int()> getSpeedCorrection,
        std::function<void()> doBeforeLinearMovement,
        std::function<bool()> obstacleIsClose,
        int mouvementSpeed);
    void goToPositionBackward(
        std::pair<float, float> targetPosition,
        std::function<std::pair<float, float>()> getCurrentPosition,
        std::function<float()> getCurrentAngle,
        std::function<int()> getSpeedCorrection,
        std::function<void()> doBeforeLinearMovement,
        std::function<bool()> obstacleIsClose,
        int mouvementSpeed);
    void goForward(
        std::function<int()> getSpeedCorrection,
        std::function<void()> doBeforeLinearMovement,
        float duration);
    void goBackward(
        std::function<int()> getSpeedCorrection,
        std::function<void()> doBeforeLinearMovement,
        float duration);

private:
    // Constants
    static const int MAX_SPEED;
    static const int MIN_SPEED;
    static const int MOUVEMENT_SPEED;
    static const int ROTATION_SPEED;
    static const float ROTATION_ANGLE_TOLERANCE;
    static const int MOUVEMENT_TOLERANCE;

    MotorsDriver driver;
};
