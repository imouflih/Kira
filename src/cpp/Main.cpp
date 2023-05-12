#include "Coordinator.hpp"
#include "../Commun/Logger/Logger.hpp"
#include "Coordinates/CoordinatesReader.hpp"
#include "Jack/Jack.hpp"
#include "EmergencyButton/EmergencyButton.hpp"
#include "ToggleLED/ToggleLED.hpp"
#include "Actuators/Actuators.hpp"
#include <iostream>
#include <unistd.h>
#include <vector>
#include <cstring>
#include <cmath>
#include <thread>
#include <atomic>
#include <chrono>

const char* ARDUINO_BOARD = "arduino:avr:nano";
const char* ARDUINO_PORT = "/dev/ttyUSB0";
const char* ARDUINO_FILENAME = "./ino/ArduinoMotors.ino";

enum Order {
    MOVE_TO,
    MOVE_TO_BACKWARD,
    FORWARD,
    BACKWARD,
    ROTATE,
    TRANSLATOR,
    ELEVATOR,
    RIGHT_CLAMP,
    LEFT_CLAMP,
    INVALID
};

void checkEmergencyButton(EmergencyButton& emergencyButton, Coordinator& coordinator, ToggleLED& toggleLED, std::atomic_bool& emergencyButtonPressed, std::atomic_bool& stopThread) {
    while (!stopThread.load()) {
        // Use the isARUpressed function from the EmergencyButton class
        if (emergencyButton.isEmergencyButtonPressed()) {
            emergencyButtonPressed.store(true);
            toggleLED.TurnOff();
            coordinator.stop();
            exit(0);
        };

        // Sleep for a short duration to prevent excessive CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

Order stringToOrder(const std::string& order) {
    if (order == "MOVE_TO") {
        return MOVE_TO;
    }
    else if (order == "MOVE_TO_BACKWARD") {
        return MOVE_TO_BACKWARD;
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
    else if (order == "TRANSLATOR") {
        return TRANSLATOR;
    }
    else if (order == "ELEVATOR") {
        return ELEVATOR;
    }
    else if (order == "RIGHT_CLAMP") {
        return RIGHT_CLAMP;
    }
    else if (order == "LEFT_CLAMP") {
        return LEFT_CLAMP;
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
    std::atomic_bool stopThread(false);

    try {
        Coordinator coordinator = Coordinator();
        Jack jack = Jack();
        EmergencyButton emergencyButton = EmergencyButton();
        ToggleLED toggleLED = ToggleLED();

        std::atomic_bool emergencyButtonPressed(false);
        std::thread emergencyButtonThread(checkEmergencyButton, std::ref(emergencyButton), std::ref(coordinator), std::ref(toggleLED), std::ref(emergencyButtonPressed), std::ref(stopThread));

        // Define our actuators
        Actuators leftClamp(1);
        Actuators rightClamp(2);
        Actuators translator(3);
        Actuators elevator(4);

        if (argc == 2) {
            if (strcmp(argv[1], "-u") == 0)
            {
                upload();
            }
            else if (strcmp(argv[1], "-s") == 0) {
                coordinator.stop();
                toggleLED.TurnOff();
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

        // Turn on the toggle led and wait for jack to be re
        toggleLED.TurnOn();
        std::cout << "Checking if the Jack is removed" << std::endl;
        while (!jack.isJackRemoved()) {
            usleep(100'000);
        }
        std::cout << "The Jack is removed, The Robot is going to start" << std::endl;
        usleep(100'000);

        for (const auto& order : orders) {
            if (emergencyButtonPressed.load()) {
                break;
            }
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
            case MOVE_TO_BACKWARD:
                std::cout << "Moving to (" << targetPosition.first << ", " << targetPosition.second << ")" << std::endl;
                coordinator.goToPositionBackward(targetPosition);
                break;
            case FORWARD:
                std::cout << "Moving forward" << std::endl;
                coordinator.goForward(angle);
                break;
            case BACKWARD:
                std::cout << "Moving backward" << std::endl;
                coordinator.goBackward(angle);
                break;
            case ROTATE:
                std::cout << "Rotating to " << angle << " radians" << std::endl;
                coordinator.rotate(angle);
                break;
            case TRANSLATOR:
                std::cout << "Action to do : Moving the Horizental Translator " << std::endl;
                translator.setGoalPosition(angle);
                sleep(3);
                break;
            case ELEVATOR:
                std::cout << "Action to do : Moving  the ELEVATOR " << angle << std::endl;
                elevator.setGoalPosition(angle);
                break;
            case RIGHT_CLAMP:
                std::cout << "Action to do : Moving the RIGHT_CLAMP " << std::endl;
                rightClamp.setGoalPosition(angle);
                break;
            case LEFT_CLAMP:
                std::cout << "Action to do : Moving the LEFT_CLAMP " << std::endl;
                leftClamp.setGoalPosition(angle);
                break;
            default:
                std::cout << "Invalid order: " << std::get<0>(order) << std::endl;
                break;
            }
        }

        toggleLED.TurnOff();
        stopThread.store(true);
        emergencyButtonThread.join();
    }
    catch (const char* msg) {
        std::cerr << "Caught exception: " << msg << std::endl;
    }
    catch (...) {
        std::cerr << "Caught an unknown exception" << std::endl;
    }
    return 0;
}
