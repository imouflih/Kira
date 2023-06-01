#include <wiringPi.h>
#include <iostream>

class DisguiseLED {
public:
    DisguiseLED();
    void TurnOn();
    void TurnOff();

private:
    static const int DISGUISE_LED_PIN;
};