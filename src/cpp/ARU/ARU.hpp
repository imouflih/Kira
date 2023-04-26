#include <wiringPi.h>
#include <iostream>

class ARU {
public:
    ARU();
    int isARUpressed();

private:
    static const int ARU_PIN;
};