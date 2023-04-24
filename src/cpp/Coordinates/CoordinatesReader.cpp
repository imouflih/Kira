#include "CoordinatesReader.hpp"

#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>

CoordinatesReader::CoordinatesReader(const std::string& filename): filename(filename) {}

std::vector<std::tuple<std::string, int, int, double>> CoordinatesReader::getCoordinates() const {
    std::ifstream input_file(filename);
    if (!input_file.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    std::vector<std::tuple<std::string, int, int, double>> orders;
    std::string line;
    std::string order;
    int x, y;
    double angle;

    while (std::getline(input_file, line)) {
        if (line.find("order") != std::string::npos) {
            std::stringstream ss(line.substr(line.find(':') + 2));
            ss >> std::quoted(order);
        }
        else if (line.find("x") != std::string::npos) {
            std::stringstream ss(line.substr(line.find(':') + 1));
            ss >> x;
        }
        else if (line.find("y") != std::string::npos) {
            std::stringstream ss(line.substr(line.find(':') + 1));
            ss >> y;
        }
        else if (line.find("angle") != std::string::npos) {
            std::stringstream ss(line.substr(line.find(':') + 1));
            ss >> angle;
        }
        else if (line.find('}') != std::string::npos) {
            orders.emplace_back(order, x, y, angle);
            x = 0;
            y = 0;
            angle = 0.0;
        }
    }

    return orders;
}
