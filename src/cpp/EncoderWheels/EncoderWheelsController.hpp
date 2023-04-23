#include "EncoderWheelsDriver.hpp"

class EncoderWheelsController {
public:
    class Coordinates {
    public:
        Coordinates(std::pair<float, float> position, float orientationAngle);
        std::pair<float, float> position;
        float orientationAngle;
    };
    EncoderWheelsController(Coordinates initialCoordinates = Coordinates(std::make_pair(.0f, .0f), .0f));
    void initCounters();
    Coordinates updateAndGetCoordinates();
    int getCountersDifference();
    int computeWheelsCorrection(int differenceCounts);

private:
    static const unsigned short ENTRAXE;       // distance between the wheels in mm
    static const unsigned short DIAMETER;       // diameter of the wheels in mm
    static const unsigned short TICK_PER_TURN; // number of ticks per wheel rotation
    static const float PROPORTIONAL_GAIN; // Proportional gain

    EncoderWheelsDriver driver;

    std::pair<int, int> previousCounters;
    Coordinates coordinates;

    float calculateWheelDistance(int currentCounter, int previousCounter) const;
    float calculateAverageDistance(float distanceLeft, float distanceRight) const;
    float calculateAngleTurned(float distanceLeft, float distanceRight) const;
    void updateCoordinates(float distanceAverage, float angle);
    float normalizeOrientationAngle(float angle) const;

};
