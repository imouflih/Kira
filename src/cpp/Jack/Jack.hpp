#include <wiringPi.h>
#include <iostream>

class Jack {
public:
    Jack();
    int read();

private:
    static const int JACK_PIN;
};