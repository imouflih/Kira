#include <wiringPi.h>
#include <iostream>

class ToggleLED {
public:
    ToggleLED();
    void TurnOn();
    void TurnOff();

private:
    static const int TOGGLE_LED_PIN;
};