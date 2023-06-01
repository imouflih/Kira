#include "EncoderWheelsDriver.hpp"

// The EncoderWheelController class provides higher-level control over the encoder wheel
class EncoderWheelsController {
public:
    // Inner class for Coordinates
    class Coordinates {
    public:
        Coordinates(std::pair<float, float> position, float orientationAngle);
        std::pair<float, float> position;
        float orientationAngle;
    };
    EncoderWheelsController(Coordinates initialCoordinates = Coordinates(std::make_pair(.0f, .0f), .0f));
    void init();
    Coordinates updateAndGetCoordinates();
    int getCountersDifference();
    int computeWheelsCorrection(int differenceCounts);

private:
    static const unsigned short ENTRAXE;        // Distance between two encoder wheels in mm
    static const unsigned short DIAMETER;       // Diameter of the encoder wheels in mm
    static const unsigned short TICK_PER_TURN;  // Number of ticks per a complete turn of the encoder wheels
    static const float PROPORTIONAL_GAIN;       // Proportional gain

    EncoderWheelsDriver driver;                 // Instance of the wheel encoder driver

    std::pair<int, int> previousCounters;
    Coordinates coordinates;

    float calculateWheelDistance(int currentCounter, int previousCounter) const;
    float calculateAverageDistance(float distanceLeft, float distanceRight) const;
    float calculateAngleTurned(float distanceLeft, float distanceRight) const;
    void updateCoordinates(float distanceAverage, float angle);
    float normalizeOrientationAngle(float angle) const;

};
