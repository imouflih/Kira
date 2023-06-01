#include "Tempo.hpp"
#include <iostream>

Tempo::Tempo(unsigned delay) :
    storedTime(std::chrono::steady_clock::now()),
    delay(delay) {}

Tempo::~Tempo() {}

// Function to store the current time
void Tempo::store() {
    this->storedTime = std::chrono::steady_clock::now();
}

// Function to check if the delay has passed since the stored time
bool Tempo::hasPassed() {
    auto currentTime = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - this->storedTime).count();
    return duration > this->delay;
}