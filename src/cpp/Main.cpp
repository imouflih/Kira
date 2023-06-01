#include "Coordinator.hpp"
#include "../Commun/Logger/Logger.hpp"
#include "Coordinates/CoordinatesReader.hpp"
#include "Jack/Jack.hpp"
#include "EmergencyButton/EmergencyButton.hpp"
#include "ToggleLED/ToggleLED.hpp"
#include "DisguiseLED/DisguiseLED.hpp"
#include "Actuators/Actuators.hpp"
#include <iostream>
#include <unistd.h>
#include <vector>
#include <cstring>
#include <cmath>
#include <thread>
#include <atomic>
#include <chrono>

// Arduino's constants
const char* ARDUINO_BOARD = "arduino:avr:nano";
const char* ARDUINO_PORT = "/dev/ttyUSB0";
const char* ARDUINO_FILENAME = "./ino/ArduinoMotors.ino";

// Match time constant
const int MATCH_TIME = 100;

// enum of all orders possible
enum Order {
    MOVE_TO,
    MOVE_TO_BACKWARD,
    MOVE_TO_AND_MODIFY_SPEED,
    MOVE_TO_BACKWARD_AND_MODIFY_SPEED,
    FORWARD,
    BACKWARD,
    ROTATE,
    TRANSLATOR,
    ELEVATOR,
    RIGHT_CLAMP,
    LEFT_CLAMP,
    RIGHT_DISTRIBUTER,
    LEFT_DISTRIBUTER,
    RESERVOIR,
    INVALID
};

// check if emergency stop button is pressed or match time has depassed 
void checkEmergencyButtonAndTimeManager(EmergencyButton& emergencyButton, Coordinator& coordinator, ToggleLED& toggleLED, DisguiseLED& disguiseLED, std::atomic_bool& stopThread, std::chrono::_V2::steady_clock::time_point& startTime) {
    while (!stopThread.load()) {
        // Check the emergency stop button
        if (emergencyButton.isEmergencyButtonPressed()) {
            toggleLED.TurnOff();
            emergencyButton.stopTheRobot();
            coordinator.stopLidar();
            exit(0);
        };
        // Get the current time
        std::chrono::_V2::steady_clock::time_point currentTime = std::chrono::steady_clock::now();

        // Calculate the time that has passed after removing the jack
        int diff = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count();
        std::cout << "Temps passé : " << diff << std::endl;

        // Stopping immediatly the robot after passing the match time
        if (diff >= MATCH_TIME) {
            emergencyButton.stopTheRobot();
            disguiseLED.TurnOn();
            toggleLED.TurnOff();
            coordinator.stopLidar();
            exit(0);
        }

        // Sleep for a short duration to prevent excessive CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// Function to convert string to an order
Order stringToOrder(const std::string& order) {
    if (order == "MOVE_TO") {
        return MOVE_TO;
    }
    else if (order == "MOVE_TO_BACKWARD") {
        return MOVE_TO_BACKWARD;
    }
    else if (order == "MOVE_TO_AND_MODIFY_SPEED") {
        return MOVE_TO_AND_MODIFY_SPEED;
    }
    else if (order == "MOVE_TO_BACKWARD_AND_MODIFY_SPEED") {
        return MOVE_TO_BACKWARD_AND_MODIFY_SPEED;
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
    else if (order == "RIGHT_DISTRIBUTER") {
        return RIGHT_DISTRIBUTER;
    }
    else if (order == "LEFT_DISTRIBUTER") {
        return LEFT_DISTRIBUTER;
    }
    else if (order == "RESERVOIR") {
        return RESERVOIR;
    }
    else {
        return INVALID;
    }
}

// To upload arduino code
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
    // Create the log file in logs folder where all logs gonna be stocked
    Logger::getInstance();
    std::atomic_bool stopThread(false);

    try {
        // Declaration of all elements of the robot
        Coordinator coordinator = Coordinator();
        Jack jack = Jack();
        EmergencyButton emergencyButton = EmergencyButton();
        ToggleLED toggleLED = ToggleLED();
        DisguiseLED disguiseLED = DisguiseLED();

        // Define our actuators
        Actuators leftClamp(1);
        Actuators rightClamp(2);
        Actuators translator(3);
        Actuators elevator(4);
        Actuators rightDistributer(5);
        Actuators leftDistibuter(6);
        Actuators reservoir(7);

        // Make sure the disguise LED is turned off
        disguiseLED.TurnOff();

        if (argc == 2) {
            // Use "./Kira -u" to upload arduino's code in the arduino
            if (strcmp(argv[1], "-u") == 0)
            {
                upload();
            }
            // Use "./Kira -s" to stop the robot
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

        // Initialization of encoder wheels and the lidar
        coordinator.init();

        // Setting all actuators to the initial position
        elevator.setGoalPosition(560);
        translator.setGoalPosition(575);
        leftClamp.setGoalPosition(700);
        rightClamp.setGoalPosition(700);
        rightDistributer.setGoalPosition(45);
        leftDistibuter.setGoalPosition(144);
        reservoir.setGoalPosition(920);

        // Reading the order from Coordinates.json file in json folder and stock it in a vector named orders
        CoordinatesReader reader("./json/Coordinates.json");
        std::vector<std::tuple<std::string, int, int, float, float, int, int>> orders = reader.getCoordinates();

        // Turn on the toggle led and wait for jack to be removed
        toggleLED.TurnOn();
        std::cout << "Checking if the Jack is removed" << std::endl;
        while (!jack.isJackRemoved()) {
            // Avoid overloading the raspberry
            usleep(100'000);
        }
        std::cout << "The Jack is removed, The Robot is going to start" << std::endl;

        // Start the chrono of the match
        std::chrono::_V2::steady_clock::time_point startTime = std::chrono::steady_clock::now();

        // Start the thread of emergency stop button and time manager
        std::thread emergencyButtonAndTimeManagerThread(checkEmergencyButtonAndTimeManager, std::ref(emergencyButton), std::ref(coordinator), std::ref(toggleLED), std::ref(disguiseLED), std::ref(stopThread), std::ref(startTime));

        // To avoid the case where the jack is slowly removed from the robot
        usleep(100'000);

        // Start actions and moves
        for (const auto& order : orders) {
            // Getting the variables of the order
            Order _order = stringToOrder(std::get<0>(order));
            std::pair<int, int> targetPosition;
            targetPosition.first = std::get<1>(order);
            targetPosition.second = std::get<2>(order);
            float angle = std::get<3>(order);
            float time = std::get<4>(order);
            int actuatorPosition = std::get<5>(order);
            int mouvementSpeed = std::get<6>(order);

            switch (_order) {
            case MOVE_TO:
                std::cout << "Moving to (" << targetPosition.first << ", " << targetPosition.second << ")" << std::endl;
                coordinator.goToPosition(targetPosition);
                break;
            case MOVE_TO_BACKWARD:
                std::cout << "Moving to (" << targetPosition.first << ", " << targetPosition.second << ")" << std::endl;
                coordinator.goToPositionBackward(targetPosition);
                break;
            case MOVE_TO_AND_MODIFY_SPEED:
                std::cout << "New speed : " << mouvementSpeed << std::endl;
                std::cout << "Moving to (" << targetPosition.first << ", " << targetPosition.second << ")" << std::endl;
                coordinator.goToPosition(targetPosition, mouvementSpeed);
                break;
            case MOVE_TO_BACKWARD_AND_MODIFY_SPEED:
                std::cout << "New speed : " << mouvementSpeed << std::endl;
                std::cout << "Moving to (" << targetPosition.first << ", " << targetPosition.second << ")" << std::endl;
                coordinator.goToPositionBackward(targetPosition, mouvementSpeed);
                break;
            case FORWARD:
                std::cout << "Moving forward for : " << time << " secondes" << std::endl;
                coordinator.goForward(time);
                break;
            case BACKWARD:
                std::cout << "Moving backward for : " << time << " secondes" << std::endl;
                coordinator.goBackward(time);
                break;
            case ROTATE:
                std::cout << "Rotating to " << angle << " radians" << std::endl;
                coordinator.rotate(angle);
                break;
            case TRANSLATOR:
                std::cout << "Action to do : Moving the Horizental Translator to " << actuatorPosition << std::endl;
                translator.setGoalPosition(actuatorPosition);
                sleep(1);
                break;
            case ELEVATOR:
                std::cout << "Action to do : Moving  the ELEVATOR to " << actuatorPosition << std::endl;
                elevator.setGoalPosition(actuatorPosition);
                usleep(750000);
                break;
            case RIGHT_CLAMP:
                std::cout << "Action to do : Moving the RIGHT_CLAMP to " << actuatorPosition << std::endl;
                rightClamp.setGoalPosition(actuatorPosition);
                break;
            case LEFT_CLAMP:
                std::cout << "Action to do : Moving the LEFT_CLAMP to " << actuatorPosition << std::endl;
                leftClamp.setGoalPosition(actuatorPosition);
                break;
            case RIGHT_DISTRIBUTER:
                std::cout << "Action to do : Moving the RIGHT_DISTRIBUTER to " << actuatorPosition << std::endl;
                rightDistributer.setGoalPosition(actuatorPosition);
                sleep(1);
                break;
            case LEFT_DISTRIBUTER:
                std::cout << "Action to do : Moving the LEFT_DISTRIBUTER to " << actuatorPosition << std::endl;
                leftDistibuter.setGoalPosition(actuatorPosition);
                sleep(1);
                break;
            case RESERVOIR:
                std::cout << "Action to do : Moving the RESERVOIR to " << actuatorPosition << std::endl;
                reservoir.setGoalPosition(actuatorPosition);
                sleep(1);
                break;
            default:
                std::cout << "Invalid order: " << std::get<0>(order) << std::endl;
                break;
            }
        }

        sleep(3);

        // Turn on the disguise LEDs
        disguiseLED.TurnOn();

        // Turn off the toggle LED
        toggleLED.TurnOff();

        // Join the emergency button and time manager thread and stop it
        stopThread.store(true);
        emergencyButtonAndTimeManagerThread.join();

        // Stopping the Lidar
        coordinator.stopLidar();
    }
    catch (const char* msg) {
        std::cerr << "Caught exception: " << msg << std::endl;
    }
    catch (...) {
        std::cerr << "Caught an unknown exception" << std::endl;
    }
    return 0;
}
