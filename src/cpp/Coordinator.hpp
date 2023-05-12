#include "../Motors/MotorsController.hpp"
#include "../EncoderWheels/EncoderWheelsController.hpp"

class Coordinator {
public:
    Coordinator();
    void goToPosition(std::pair<int, int> targetPosition, int mouvementSpeed = MOUVEMENT_SPEED);
    void goToPositionBackward(std::pair<int, int> targetPosition, int mouvementSpeed = MOUVEMENT_SPEED);
    void init();
    void stop();
    void rotate(float targetAngle);
    void goForward(int duration);
    void goBackward(int duration);

private:
    static const int MOUVEMENT_SPEED;

    MotorsController motorsController;
    EncoderWheelsController encoderWheelsController;
    int initialCountersDifference;

    std::pair<float, float> getCurrentPosition();
    float getOrientation();
    int getSpeedCorrection();
    void updateInitialCountersDifference();
};
