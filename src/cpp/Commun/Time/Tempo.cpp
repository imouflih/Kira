#include "Tempo.hpp"
#include <iostream>

Tempo::Tempo(unsigned delay) :
    storedTime(std::chrono::steady_clock::now()),
    delay(delay) {}

Tempo::~Tempo() {}

void Tempo::store() {
    this->storedTime = std::chrono::steady_clock::now();
}

bool Tempo::hasPassed() {
    auto currentTime = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - this->storedTime).count();
    return duration > this->delay;
}