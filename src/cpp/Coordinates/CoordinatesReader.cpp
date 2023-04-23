#include "CoordinatesReader.hpp"

#include <fstream>
#include <sstream>
#include <iostream>

CoordinatesReader::CoordinatesReader(const std::string& filename): filename(filename) {}

std::vector<std::pair<int, int>> CoordinatesReader::getCoordinates() const {
    std::ifstream input_file(filename);
    if (!input_file.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    std::vector<std::pair<int, int>> pairs;
    std::string line;
    int x, y;

    while (std::getline(input_file, line)) {
        if (line.find("x") != std::string::npos) {
            std::stringstream ss(line.substr(line.find(':') + 1));
            ss >> x;
        }
        else if (line.find("y") != std::string::npos) {
            std::stringstream ss(line.substr(line.find(':') + 1));
            ss >> y;
            pairs.emplace_back(x, y);
        }
    }

    return pairs;
}
