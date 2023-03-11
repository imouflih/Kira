/*
   This program is used to control the motors for Kira
   see https://github.com/imouflih/Kira

   It uses an I2C bus to communicate with the RapsberryPi.
   The board send a PWM value between -255 and 255 for each motors.

   Last updated : February 26 2022
   Author : Iliasse Mouflih
*/

#include <iostream>
#include <cstring>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

// Maximum speed
#define MAX_SPEED   30

const char* BOARD = "arduino:avr:nano"; // Replace with your board type
const char* PORT = "/dev/ttyUSB0"; // Replace with your Arduino port name
const char* I2C_BUS = "/dev/i2c-1"; // Replace with your I2C bus name
const int ADDRESS = 0x0C; // Replace with your I2C address
const char* FILENAME = "../ino/ArduinoMotors.ino"; // Replace with your arduino file

enum Direction {
  STOPPED = 0,
  FORWARD = 1,
  BACKWARD = 2
};

int sendRequest() {
  int file = open(I2C_BUS, O_RDWR);
  if (file < 0) {
    printf("Failed to open I2C device.\n");
    return -1;
  }

  if (ioctl(file, I2C_SLAVE, ADDRESS) < 0) {
    printf("Failed to set I2C address.\n");
    return -1;
  }

  // Send request to Arduino to send data back
  if (write(file, "1", 1) != 1) {
    printf("Failed to write to I2C bus.\n");
    return -1;
  }

  // Read data from Arduino
  char buf[15];
  int num_bytes = read(file, buf, sizeof(buf));
  if (num_bytes < 0) {
    printf("Failed to read from I2C bus.\n");
    return -1;
  }

  // Print received data
  printf("Received data: %s\n", buf);

  close(file);
  return 0;
}

int upload() {
  // Open the Arduino IDE and create a new process
  std::string command = "arduino --board " + std::string(BOARD) + " --port " + std::string(PORT) + " --upload " + std::string(FILENAME);
  int result = system(command.c_str());

  if (result == 0) {
    std::cout << "Upload successful" << std::endl;
  }
  else {
    std::cout << "Upload failed" << std::endl;
  }

  return result;
}

void setOrder(int leftOrder, int rightOrder) {

  // The speed is a positive value between 0 and 255
  int leftSpeed = (uint8_t)abs(leftOrder);
  int rightSpeed = (uint8_t)abs(rightOrder);

  // The direction is defined by the sign
  Direction leftDirection = leftOrder == 0 ? STOPPED : leftOrder > 0 ? FORWARD : BACKWARD;
  Direction rightDirection = rightOrder == 0 ? STOPPED : rightOrder > 0 ? FORWARD : BACKWARD;

  // Send the data
  uint8_t data[4];
  data[0] = (uint8_t)rightSpeed > MAX_SPEED ? MAX_SPEED : (uint8_t)rightSpeed;
  data[1] = (uint8_t)leftSpeed > MAX_SPEED ? MAX_SPEED : (uint8_t)leftSpeed;
  data[2] = (uint8_t)rightDirection;
  data[3] = (uint8_t)leftDirection;
  std::cout << "[MOTOR DEBUG] Printing the content of data :" << std::endl;
  std::cout << "[MOTOR DEBUG] data[0] : " << unsigned(data[0]) << std::endl;
  std::cout << "[MOTOR DEBUG] data[1] : " << unsigned(data[1]) << std::endl;
  std::cout << "[MOTOR DEBUG] data[2] : " << unsigned(data[2]) << std::endl;
  std::cout << "[MOTOR DEBUG] data[3] : " << unsigned(data[3]) << std::endl;

  // Sending the byte array
  int i2c_fd = open(I2C_BUS, O_RDWR);
  if (i2c_fd < 0) {
    printf("Failed to open I2C device.\n");
    return;
  }

  if (ioctl(i2c_fd, I2C_SLAVE, ADDRESS) < 0) {
    printf("Failed to set I2C address.\n");
    return;
  }
  if (write(i2c_fd, data, 4) == -1) {
    printf("Failed to write to I2C bus.\n");
    return;
  }
}

int main(int argc, char** argv) {
  if (argc == 2) {
    if (strcmp(argv[1], "-u") == 0)
    {
      upload();
    }
    else
    {
      printf("Wrong Command Format !\n");
    }
    return -1;
  }
  if (argc == 3) {
    int leftOrder = atoi(argv[1]);
    int rightOrder = atoi(argv[2]);
    setOrder(leftOrder, rightOrder);
  }
  // sendRequest();
  return 0;
}