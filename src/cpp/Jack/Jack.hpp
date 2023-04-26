#include <wiringPi.h>
#include <iostream>

class Jack {
public:
    Jack();
    int isJackRemoved();

private:
    static const int JACK_PIN;
};