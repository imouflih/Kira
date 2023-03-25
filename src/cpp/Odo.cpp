#include <iostream>
#include <termios.h>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <cstdint>
#include <cmath>

#define ADDRESS_I2C 12
#define START_BYTE 0xA5
#define STOP_BYTE 0x5A
#define INIT_COUNTERS 0xA1

// Odometry config
#define ENTRAXE 235       // distance between the wheels in mm
#define DIAMETER 35       // diameter of the wheels in mm
#define TICK_PER_TURN 512 // number of ticks per wheel rotation

// Control config
#define THETA_TOLERANCE 0.03 // rad

#define MAX_SPEED 50
#define MIN_SPEED 7

const char* I2C_BUS = "/dev/i2c-1"; // I2C bus
const float Kp = 0.1; // Proportional gain

int file = open(I2C_BUS, O_RDWR);

int countLeft;
int countRight;

// Global variables
float x;                   // x position of the robot
float y;                   // y position of the robot
float theta;             // orientation of the robot
int countLeft_previous;  // previous count of left encoder
int countRight_previous; // previous count of right encoder

int speed;

uint8_t data[10];
uint8_t data_to_send[4];

void init() {
    unsigned char b = INIT_COUNTERS;
    if (write(file, &b, 1) == -1) {
        printf("Failed to write to I2C bus. -init\n");
    }
    x = y = theta = 0;
    countLeft_previous = countRight_previous = 0;
}

void orderMove(int speedRight, int speedLeft) {
    // make sure speed doesn't exceed the max and min speed
    speedRight = (speedRight != 0) ? ((speedRight > 0) ? 1 : -1) * std::max(MIN_SPEED, std::min(abs(speedRight), MAX_SPEED)) : 0;
    speedLeft = (speedLeft != 0) ? ((speedLeft > 0) ? 1 : -1) * std::max(MIN_SPEED, std::min(abs(speedLeft), MAX_SPEED)) : 0;

    // send data to arduino
    data_to_send[0] = speedLeft & 0xFF;
    data_to_send[1] = (speedLeft >> 8) & 0xFF;
    data_to_send[2] = speedRight & 0xFF;
    data_to_send[3] = (speedRight >> 8) & 0xFF;

    if (file < 0) {
        printf("Failed to open I2C device.\n");
        return;
    }
    if (ioctl(file, I2C_SLAVE, ADDRESS_I2C) < 0) {
        printf("Failed to set I2C address.\n");
        return;
    }
    if (write(file, data_to_send, 4) == -1) {
        printf("Failed to write to I2C bus. -orderMove\n");
        return;
    }
}

void getCounts() {
    if (read(file, data, 10) != 10) {
        perror("Failed to read data from i2c bus. -getCounts");
        exit(1);
    }

    if (data[0] != START_BYTE || data[9] != STOP_BYTE) {
        std::cout << "Error: Incorrect data" << std::endl;
    }

    countLeft = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24);
    countRight = data[5] | (data[6] << 8) | (data[7] << 16) | (data[8] << 24);
}

void correctorSpeed(int leftSpeed, int rightSpeed, long differenceCounts) {
    // Calculate the error between left and right encoder counts
    long error = countRight - countLeft - differenceCounts;

    // Calculate the speed correction using proportional gain
    float correction = Kp * error;

    // Apply the correction to the motor speeds
    float correctedLeftSpeed = static_cast<int>(round(leftSpeed + correction));
    float correctedRightSpeed = static_cast<int>(round(rightSpeed - correction));

    // Set the corrected motor speeds
    orderMove(correctedLeftSpeed, correctedRightSpeed);
}

// Function that update (x,y,theta) of the robot
void update_position() {

    getCounts();

    // Calculate the distance traveled by each wheel
    float distanceLeft = (countLeft - countLeft_previous) * 2 * M_PI * DIAMETER / TICK_PER_TURN;
    float distanceRight = (countRight - countRight_previous) * 2 * M_PI * DIAMETER / TICK_PER_TURN;

    // Calculate the average distance traveled by the robot
    float distanceAverage = (distanceLeft + distanceRight) / 2;

    // Calculate the angle turned by the robot
    float angle = (distanceRight - distanceLeft) / ENTRAXE;

    // Calculate the change in position of the robot
    float delta_x = distanceAverage * cos(theta + angle / 2);
    float delta_y = distanceAverage * sin(theta + angle / 2);
    float delta_theta = angle;

    // Update the position of the robot
    x += delta_x;
    y += delta_y;
    theta += delta_theta;

    // Make sure that theta is between 0 and 4*PI
    theta = (fmod(theta, 4 * M_PI) < 0 ? fmod(theta, 4 * M_PI) + 4 * M_PI : fmod(theta, 4 * M_PI));

    // Update the previous count values
    countLeft_previous = countLeft;
    countRight_previous = countRight;

    std::printf("\033[2J"); // Clear the screen
    std::printf("\033[0;0H"); // Move the cursor to the top-left corner
    std::printf("countLeft = %d, countRight = %d", countLeft, countRight);
    std::printf("\nx = %.2f, y = %.2f, theta = %.2f\n", x, y, theta);
    std::fflush(stdout); // Flush stdout to ensure the output is printed immediately

    // sleep(0.1);

}

// Function to rotate the robot
void setTheta(float theta_target) {

    int speed = 15;

    // Get the current angle of the robot
    update_position();

    // Rotate the robot, make sure that he always takes the shortest path
    fabs(theta_target - theta) < 2 * M_PI ? theta_target > theta ? orderMove(-speed, speed) : orderMove(speed, -speed) : theta_target > theta ? orderMove(speed, -speed) : orderMove(-speed, speed);

    // Continue adjusting the angle until it is within the specified tolerance
    while (fmod(fabs(theta_target - theta), 4 * M_PI) > THETA_TOLERANCE) {

        // Update the current angle of the robot
        update_position();
    }

    orderMove(0, 0); // Stop the robot
}

void go_to(int x_target, int y_target) {
    // Update the current position
    update_position();
    int x_current = x;
    int y_current = y;
    float angle_init = theta;

    // Calculate the distance and angle between the current position and target position
    int dx = x_target - x_current;
    int dy = y_target - y_current;
    float distance = sqrt(dx * dx + dy * dy);
    float angle = atan2(dy, dx);
    angle = fmod(angle, 4 * M_PI) < 0 ? fmod(angle, 4 * M_PI) + 4 * M_PI : fmod(angle, 4 * M_PI);

    // Turn the robot towards the target position
    setTheta(angle);

    update_position();

    long diff = countRight - countLeft;

    orderMove(speed, speed);

    // Move the robot forward towards the target position
    while (distance > 30) {
        // Update the current position
        update_position();
        x_current = x;
        y_current = y;

        // Calculate the new distance and angle to the target position
        dx = x_target - x_current;
        dy = y_target - y_current;
        distance = sqrt(dx * dx + dy * dy);
        correctorSpeed(speed, speed, diff);
        // angle = atan2(dy, dx) * 180.0 / M_PI;
        // angle = fmod(angle, 2 * M_PI) < 0 ? fmod(angle, 2 * M_PI) + 2 * M_PI : fmod(angle, 2 * M_PI);

        // // Turn the robot towards the target position
        // setTheta(angle);
        // orderMove(speed, speed);
    }

    // Stop the robot
    orderMove(0, 0);

    // Rotate the robot to the initial angle
    setTheta(angle_init);
}

int main(int argc, char** argv) {
    if (file < 0) {
        perror("Failed to open i2c bus");
        exit(1);
    }

    if (ioctl(file, I2C_SLAVE, ADDRESS_I2C) < 0) {
        perror("Failed to acquire bus access or talk to slave");
        exit(1);
    }
    speed = 50;
    if (argc == 3) {
        // init();
        sleep(1);
        int speedLeft, speedRight;
        speedLeft = atoi(argv[1]);
        speedRight = atoi(argv[2]);
        orderMove(speedLeft, speedRight);
        update_position();
        long diff = countRight - countLeft;
        sleep(3);
        while (true) {
            correctorSpeed(speedLeft, speedRight, diff);
            update_position();
        }
        return 0;
    }
    if (argc == 4) {
        init();
        sleep(1);
        int speedLeft, speedRight;
        speedLeft = atoi(argv[1]);
        speedRight = atoi(argv[2]);
        orderMove(speedLeft, speedRight);
        printf("speed left : %d\nspeed right : %d\n", speedLeft, speedRight);
        return 0;
    }
    init();
    sleep(1);

    go_to(5000, 0);
    return 0;
}
