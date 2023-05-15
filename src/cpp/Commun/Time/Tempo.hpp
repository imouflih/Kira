#pragma once

#include <chrono>

class Tempo {
public:
    Tempo(unsigned delay = 50);
    ~Tempo();

    void store();
    bool hasPassed();

private:
    std::chrono::steady_clock::time_point storedTime;
    unsigned delay;
};