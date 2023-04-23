#include "Coordinator.hpp"
#include "../Commun/Logger/Logger.hpp"
#include "Coordinates/CoordinatesReader.hpp"
#include <iostream>
#include <vector>
#include <cstring>

const char* ARDUINO_BOARD = "arduino:avr:nano";
const char* ARDUINO_PORT = "/dev/ttyUSB0";
const char* ARDUINO_FILENAME = "./ino/ArduinoMotors.ino";

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

        std::vector<std::pair<int, int>> coordinates = reader.getCoordinates();
        for (const std::pair<int, int> targetPosition : coordinates) {
            std::cout << "New Target :(" << targetPosition.first << ", " << targetPosition.second << ")" << std::endl;
            coordinator.goToPosition(targetPosition);
        }
    }
    catch (const char* msg) {
        std::cerr << "Caught exception: " << msg << std::endl;
    }
    catch (...) {
        std::cerr << "Caught an unknown exception" << std::endl;
    }
    return 0;
}
