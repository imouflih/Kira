#include "../Motors/MotorsController.hpp"
#include "../EncoderWheels/EncoderWheelsController.hpp"

class Coordinator {
public:
    Coordinator();
    void goToPosition(std::pair<int, int> targetPosition);
    void goToPositionBackward(std::pair<int, int> targetPosition);
    void init();
    void stop();
    void rotate(float targetAngle);
    void goForward();
    void goBackward();

private:
    MotorsController motorsController;
    EncoderWheelsController encoderWheelsController;
    int initialCountersDifference;

    std::pair<float, float> getCurrentPosition();
    float getOrientation();
    int getSpeedCorrection();
    void updateInitialCountersDifference();
};
