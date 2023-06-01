#include "../Commun/GenericDriver.hpp"

// This class is used to insure communication of the motors speed values between the raspberry and the arduino (low-level control)
class MotorsDriver : GenericDriver {

public:
    MotorsDriver();
    void setMotorsSpeed(int speedRight, int speedLeft); // TODO : fix inversion

};
