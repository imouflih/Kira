#include "../Commun/GenericDriver.hpp"

class MotorsDriver: GenericDriver {

public:
    MotorsDriver();
    void setMotorsSpeed(int speedRight, int speedLeft); // todo : fix inversion

};
