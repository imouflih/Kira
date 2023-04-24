#include "Coordinator.hpp"
#include "../Commun/Logger/Logger.hpp"
#include "Coordinates/CoordinatesReader.hpp"
#include <iostream>
#include <unistd.h>
#include <vector>
#include <cstring>
#include <cmath>

const char* ARDUINO_BOARD = "arduino:avr:nano";
const char* ARDUINO_PORT = "/dev/ttyUSB0";
const char* ARDUINO_FILENAME = "./ino/ArduinoMotors.ino";

enum Order {
    MOVE_TO,
    FORWARD,
    BACKWARD,
    ROTATE,
    INVALID
};

Order stringToOrder(const std::string& order) {
    if (order == "MOVE_TO") {
        return MOVE_TO;
    }
    else if (order == "FORWARD") {
        return FORWARD;
    }
    else if (order == "BACKWARD") {
        return BACKWARD;
    }
    else if (order == "ROTATE") {
        return ROTATE;
    }
    else {
        return INVALID;
    }
}

int upload() {
    // Open the Arduino IDE and create a new process
    std::string command = "arduino --board " + std::string(ARDUINO_BOARD) + " --port " + std::string(ARDUINO_PORT) + " --upload " + std::string(ARDUINO_FILENAME);
    int result = system(command.c_str());

    if (result == 0) {
        std::cout << "Upload successful" << std::endl;
    }
    else {
        std::cout << "Upload failed" << std::endl;
    }

    return result;
}

int main(int argc, char** argv) {
    Logger::getInstance();
    try {
        Coordinator coordinator = Coordinator();

        if (argc == 2) {
            if (strcmp(argv[1], "-u") == 0)
            {
                upload();
            }
            else if (strcmp(argv[1], "-s") == 0) {
                coordinator.stop();
            }
            else
            {
                printf("Wrong Command Format !\n");
                return -1;
            }
            return 0;
        }
        coordinator.init();

        CoordinatesReader reader("./json/Coordinates.json");

        std::vector<std::tuple<std::string, int, int, double>> orders = reader.getCoordinates();
        for (const auto& order : orders) {
            Order o = stringToOrder(std::get<0>(order));
            std::pair<int, int> targetPosition;
            targetPosition.first = std::get<1>(order);
            targetPosition.second = std::get<2>(order);
            double angle = std::get<3>(order);
            switch (o) {
            case MOVE_TO:
                std::cout << "Moving to (" << targetPosition.first << ", " << targetPosition.second << ")" << std::endl;
                coordinator.goToPosition(targetPosition);
                break;
            case FORWARD:
                std::cout << "Moving forward" << std::endl;
                coordinator.goForward();
                break;
            case BACKWARD:
                std::cout << "Moving backward" << std::endl;
                coordinator.goBackward();
                break;
            case ROTATE:
                std::cout << "Rotating by " << angle << " radians" << std::endl;
                coordinator.rotate(angle);
                break;
            default:
                std::cout << "Invalid order: " << std::get<0>(order) << std::endl;
                break;
            }
            sleep(3);
        }
        // for (const std::pair<int, int> targetPosition : coordinates) {
        //     std::cout << "New Target :(" << targetPosition.first << ", " << targetPosition.second << ")" << std::endl;
        //     coordinator.goToPosition(targetPosition);
        // }
        // coordinator.rotate(2 * M_PI);
        // coordinator.goForward();
        // coordinator.goBackward();
    }
    catch (const char* msg) {
        std::cerr << "Caught exception: " << msg << std::endl;
    }
    catch (...) {
        std::cerr << "Caught an unknown exception" << std::endl;
    }
    return 0;
}
