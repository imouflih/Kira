#pragma once

#include <chrono>

// Tempo class can be used to measure whether a specified delay, made to replace sleep function without stopping the programme from running
class Tempo {
public:
    Tempo(unsigned delay = 50);
    ~Tempo();

    void store();
    bool hasPassed();

private:
    // Stores a point in time
    std::chrono::steady_clock::time_point storedTime;
    // The delay time in milliseconds.
    unsigned delay;
};